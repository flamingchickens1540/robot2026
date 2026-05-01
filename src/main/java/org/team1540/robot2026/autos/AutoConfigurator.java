package org.team1540.robot2026.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.SimState;
import org.team1540.robot2026.commands.FeedingCommands;
import org.team1540.robot2026.commands.ShootingCommands;
import org.team1540.robot2026.subsystems.drive.Drivetrain;
import org.team1540.robot2026.subsystems.hood.Hood;
import org.team1540.robot2026.subsystems.intake.Intake;
import org.team1540.robot2026.subsystems.shooter.Shooter;
import org.team1540.robot2026.subsystems.spindexer.Spindexer;
import org.team1540.robot2026.subsystems.turret.Turret;

public class AutoConfigurator {
    public enum StartingSide {
        NONE(false),
        LEFT(false),
        RIGHT(true);

        public final boolean mirrored;

        StartingSide(boolean mirrored) {
            this.mirrored = mirrored;
        }
    }

    public enum SweepType {
        NONE,
        SWEEP,
        HOOK,
        PLOW,
        STEAL
    }

    public enum SweepPath {
        NONE("", SweepType.NONE),
        CLOSE_SWEEP("CloseSweep", SweepType.SWEEP),
        FAR_SWEEP("FarSweep", SweepType.SWEEP),
        CLOSE_HOOK("CloseHook", SweepType.HOOK),
        FAR_HOOK("FarHook", SweepType.HOOK),
        CHEZY_HOOK_BUMP("ChezyHook", SweepType.HOOK),
        CLOSE_PLOW("ClosePlow", SweepType.PLOW),
        CENTER_PLOW("CenterPlow", SweepType.PLOW),
        OPPOSING_PLOW("OpposingPlow", SweepType.PLOW),
        CLOSE_PLOW_BUMP("ClosePlowBump", SweepType.PLOW),
        CENTER_PLOW_BUMP("CenterPlowBump", SweepType.PLOW),
        OPPOSING_PLOW_BUMP("OpposingPlowBump", SweepType.PLOW),
        MADTOWN_SWEEP_BUMP("MadtownSweep", SweepType.PLOW),
        OPPOSING_STEAL("OpposingSteal", SweepType.STEAL);

        public final String trajectoryName;
        public final boolean firstSweepOnly; // Can this path only be selected as the first sweep
        public final boolean alignAtEnd; // Does this path end with the robot faced away from the neutral zone
        public final boolean rotatedEnd;
        public final boolean bump;

        SweepPath(String trajectoryName, SweepType type) {
            this.trajectoryName = trajectoryName;
            firstSweepOnly = false;
            bump = toString().endsWith("BUMP");
            rotatedEnd = type == SweepType.HOOK;
            alignAtEnd = type == SweepType.HOOK || bump;
        }
    }

    public enum EndAction {
        NONE(""),
        SPRINT("Sprint"),
        DEPOT("DepotIntake"),
        DEPOT_NO_RETURN("DepotIntakeNoReturn"),
        TIME_OUT_CORNER("TimeOutCorner"),
        HUB("Hub"),
        TOWER("Tower");

        public final String trajectoryName;
        public final boolean depot;

        EndAction(String trajectoryName) {
            this.trajectoryName = trajectoryName;
            this.depot = toString().startsWith("DEPOT");
        }
    }

    private final AutoFactory autoFactory;

    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;

    private final LoggedDashboardChooser<StartingSide> startingSideChooser =
            new LoggedDashboardChooser<>("Auto/Configurator/Starting Side");
    private final LoggedDashboardChooser<SweepPath> sweep1Chooser =
            new LoggedDashboardChooser<>("Auto/Configurator/1st Sweep");
    private final LoggedDashboardChooser<SweepPath> sweep2Chooser =
            new LoggedDashboardChooser<>("Auto/Configurator/2nd Sweep");
    private final LoggedDashboardChooser<EndAction> endActionChooser =
            new LoggedDashboardChooser<>("Auto/Configurator/End Action");

    private final LoggedNetworkNumber shootTime =
            new LoggedNetworkNumber("SmartDashboard/Auto/Configurator/Shoot Time", 5.0);

    private AutoRoutineData selectedAuto;

    private final ArrayList<Consumer<AutoRoutineData>> changeListeners = new ArrayList<>();

    public AutoConfigurator(
            AutoFactory autoFactory,
            Drivetrain drivetrain,
            Intake intake,
            Spindexer spindexer,
            Turret turret,
            Hood hood,
            Shooter shooter) {
        this.autoFactory = autoFactory;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.spindexer = spindexer;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;

        startingSideChooser.addDefaultOption("Left", StartingSide.LEFT);
        startingSideChooser.addOption("Right", StartingSide.RIGHT);

        for (SweepPath path : SweepPath.values()) {
            String chooserName = path.trajectoryName.isEmpty() ? "None" : path.trajectoryName;

            if (path == SweepPath.NONE) {
                sweep1Chooser.addDefaultOption(chooserName, path);
                sweep2Chooser.addDefaultOption(chooserName, path);
            } else {
                sweep1Chooser.addOption(chooserName, path);
                if (!path.firstSweepOnly) sweep2Chooser.addOption(chooserName, path);
            }
        }

        for (EndAction action : EndAction.values()) {
            String chooserName = action.trajectoryName.isEmpty() ? "None" : action.trajectoryName;
            if (action == EndAction.NONE) {
                endActionChooser.addDefaultOption(chooserName, action);
            } else {
                endActionChooser.addOption(chooserName, action);
            }
        }

        startingSideChooser.onChange(side -> updateListeners());
        sweep1Chooser.onChange(path -> updateListeners());
        sweep2Chooser.onChange(path -> updateListeners());
        endActionChooser.onChange(action -> updateListeners());
    }

    private void updateSelectedAuto() {
        StartingSide startingSide = startingSideChooser.get();
        SweepPath sweep1 = sweep1Chooser.get();
        SweepPath sweep2 = sweep2Chooser.get();
        EndAction endAction = endActionChooser.get();

        if (startingSide == null || sweep1 == null || sweep2 == null || endAction == null) {
            return;
        }

        String name = startingSide == StartingSide.LEFT ? "LeftSide" : "RightSide";
        if (sweep1 != SweepPath.NONE) {
            name += "_" + sweep1.trajectoryName;
        }
        if (sweep2 != SweepPath.NONE) {
            name += "_" + sweep2.trajectoryName;
        }

        List<AutoTrajectory> trajectories = new ArrayList<>();

        AutoRoutine routine = autoFactory.newRoutine(name);
        routine.active()
                .onTrue(hood.zeroCommand()
                        .withTimeout(1.0)
                        .withName("ZeroHoodCommand")); // Always zero hood at start of auto
        if (sweep1 == SweepPath.NONE && sweep2 == SweepPath.NONE) {
            routine.active().onTrue(intake.zeroCommand().withTimeout(1.0)); // If no sweeps, zero intake
        }

        Trigger nextTrigger = routine.active();

        nextTrigger = addSweepRoutine(
                routine,
                sweep1,
                startingSide,
                sweep2 == SweepPath.NONE && endAction != EndAction.SPRINT,
                sweep2 != SweepPath.NONE || !endAction.depot,
                nextTrigger,
                trajectories);
        nextTrigger = addSweepRoutine(
                routine,
                sweep2,
                startingSide,
                endAction != EndAction.SPRINT,
                !endAction.depot,
                nextTrigger,
                trajectories);

        if (endAction == EndAction.SPRINT) {
            AutoTrajectory sprintTraj = routine.trajectory(endAction.trajectoryName);
            if (startingSide.mirrored) sprintTraj = sprintTraj.mirrorY();
            trajectories.add(sprintTraj);

            nextTrigger.onTrue(sprintTraj.spawnCmd().alongWith(intake.commandRunIntake(1.0)));
        } else if (endAction.depot && startingSide == StartingSide.LEFT) {
            SweepPath lastSweep = sweep2 != SweepPath.NONE ? sweep2 : sweep1;

            String trajName = endAction.trajectoryName;
            if (lastSweep.bump) trajName = "Bump" + trajName;

            AutoTrajectory depotTraj = routine.trajectory(trajName);
            if (startingSide.mirrored) depotTraj = depotTraj.mirrorY();
            trajectories.add(depotTraj);

            nextTrigger.onTrue(depotTraj.cmd());
            nextTrigger.onTrue(intake.commandRunIntake(1.0));
            depotTraj.atTime("Intake").onTrue(intake.commandRunDepotIntake(1.0).withName("AutoDepotIntakeCommand"));
            depotTraj.atTime("StopIntake").onTrue(intake.jiggleCommand());
        } else if (endAction != EndAction.NONE) {
            SweepPath lastSweep = sweep2 != SweepPath.NONE ? sweep2 : sweep1;

            String trajName = endAction.trajectoryName;
            if (!lastSweep.bump) trajName = (lastSweep.rotatedEnd ? "Rotated" : "") + "TrenchTo" + trajName;
            else trajName = "BumpTo" + trajName;

            AutoTrajectory traj = routine.trajectory(trajName);
            if (startingSide.mirrored) traj = traj.mirrorY();
            trajectories.add(traj);

            nextTrigger.onTrue(traj.cmd());
        }

        Optional<Pose2d> startingPose = trajectories.isEmpty()
                ? Optional.empty()
                : trajectories.get(0).getRawTrajectory().getInitialPose(false);
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            routine.active().onTrue(Commands.runOnce(() -> SimState.getInstance()
                    .resetForAuto(startingPose.orElse(Pose2d.kZero))));
        }

        List<SweepPath> sweeps = new ArrayList<>();
        if (sweep1 != SweepPath.NONE) sweeps.add(sweep1);
        if (sweep2 != SweepPath.NONE) sweeps.add(sweep2);

        List<Pose2d> trajectoryPoints = trajectories.stream()
                .collect(
                        ArrayList::new,
                        (list, traj) ->
                                list.addAll(List.of(traj.getRawTrajectory().getPoses())),
                        ArrayList::addAll);

        selectedAuto = new AutoRoutineData(name, startingSide, startingPose, sweeps, trajectoryPoints, routine.cmd());
    }

    private Trigger addSweepRoutine(
            AutoRoutine routine,
            SweepPath sweep,
            StartingSide startingSide,
            boolean shootIndefinitely, // Whether to keep shooting after the sweep until the end of auto
            boolean alignAtEnd, // Whether to rotate to point towards the neutral zone at the end of the sweep (only
            // applies to paths with rotated end)
            Trigger previousTrigger, // The trigger that should start this sweep routine
            List<AutoTrajectory> trajectories) {
        if (sweep == SweepPath.NONE) {
            return previousTrigger;
        }

        AutoTrajectory traj = routine.trajectory(sweep.trajectoryName, 0);
        if (startingSide.mirrored) traj = traj.mirrorY();

        trajectories.add(traj);

        previousTrigger.onTrue(traj.cmd());
        previousTrigger.onTrue(intake.zeroWhileRunningCommand()
                .andThen(intake.commandRunIntake(1.0))
                .withName("AutoZeroAndRunIntakeCommand"));

        Trigger doneTrigger = shootIndefinitely ? routine.observe(() -> false) : traj.doneDelayed(shootTime.get());

        // If the sweep ends rotated, run a rotation trajectory while shooting
        if (sweep.alignAtEnd && alignAtEnd) {
            AutoTrajectory rotateTraj = routine.trajectory(sweep.trajectoryName, 1);
            if (startingSide.mirrored) rotateTraj = rotateTraj.mirrorY();
            trajectories.add(rotateTraj);
            traj.chain(rotateTraj);
        }

        // Shoot after sweep
        traj.done()
                .onTrue(ShootingCommands.hubAimCommand(turret, shooter, hood)
                                .alongWith(
                                        FeedingCommands.feedCommand(turret, hood, spindexer),
                                        intake.jiggleCommand().asProxy())
                                .until(doneTrigger)
                                .withName("AutoShootCommand"));

        return shootIndefinitely ? traj.done() : doneTrigger;
    }

    public AutoRoutineData getSelectedAuto() {
        return selectedAuto;
    }

    public void addChangeListener(Consumer<AutoRoutineData> listener) {
        changeListeners.add(listener);
    }

    private void updateListeners() {
        updateSelectedAuto();
        changeListeners.forEach(action -> action.accept(selectedAuto));
    }
}
