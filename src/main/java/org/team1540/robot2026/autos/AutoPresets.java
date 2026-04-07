package org.team1540.robot2026.autos;

import static org.team1540.robot2026.autos.AutoConfigurator.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.RobotState;
import org.team1540.robot2026.SimState;
import org.team1540.robot2026.commands.FeedingCommands;
import org.team1540.robot2026.commands.ShootingCommands;
import org.team1540.robot2026.subsystems.drive.Drivetrain;
import org.team1540.robot2026.subsystems.hood.Hood;
import org.team1540.robot2026.subsystems.intake.Intake;
import org.team1540.robot2026.subsystems.shooter.Shooter;
import org.team1540.robot2026.subsystems.spindexer.Spindexer;
import org.team1540.robot2026.subsystems.turret.Turret;

public class AutoPresets {
    private final RobotState robotState = RobotState.getInstance();
    private final AutoFactory autoFactory;

    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;

    public AutoPresets(
            AutoFactory autoFactory,
            Drivetrain drivetrain,
            Intake intake,
            Spindexer spindexer,
            Turret turret,
            Hood hood,
            Shooter shooter) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.spindexer = spindexer;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.autoFactory = autoFactory;
    }

    private void resetPoseInSim(AutoRoutine routine, AutoTrajectory startingTrajectory) {
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            routine.active().onTrue(Commands.runOnce(() -> SimState.getInstance()
                    .resetForAuto(startingTrajectory
                            .getRawTrajectory()
                            .getInitialPose(false)
                            .orElse(new Pose2d(3, 3, Rotation2d.kZero)))));
        }
    }

    public AutoRoutineData testAuto() {
        final String trajName = "TestAuto";

        AutoRoutine routine = autoFactory.newRoutine("TestAuto");
        AutoTrajectory traj = routine.trajectory(trajName);
        routine.active()
                .onTrue(traj.cmd()
                        .beforeStarting(Commands.runOnce(
                                () -> robotState.resetPose(traj.getInitialPose().orElse(Pose2d.kZero)))));

        return new AutoRoutineData(
                "TestAuto",
                StartingSide.NONE,
                traj.getInitialPose(),
                List.of(),
                Arrays.asList(traj.getRawTrajectory().getPoses()),
                routine.cmd());
    }

    public AutoRoutineData zeroMechanisms() {
        return new AutoRoutineData(
                "ZeroMechanisms",
                StartingSide.NONE,
                Optional.empty(),
                List.of(),
                List.of(),
                Commands.parallel(intake.zeroCommand(), hood.zeroCommand()).withName("ZeroMechanisms"));
    }

    public AutoRoutineData shootPreload() {
        return new AutoRoutineData(
                "ShootPreload",
                StartingSide.NONE,
                Optional.empty(),
                List.of(),
                List.of(),
                Commands.parallel(
                                intake.zeroCommand().withTimeout(1.0).andThen(intake.jiggleCommand()),
                                hood.zeroCommand()
                                        .withTimeout(1.0)
                                        .andThen(ShootingCommands.hubAimCommand(turret, shooter, hood)
                                                .alongWith(FeedingCommands.feedCommand(turret, hood, spindexer))))
                        .withName("ShootPreload"));
    }

    public AutoRoutineData singleSweep(StartingSide startingSide, boolean sprint) {
        final String trajName = "SingleSweep";

        String name = (startingSide == StartingSide.LEFT ? "Left" : "Right") + trajName + (sprint ? "Sprint" : "");
        AutoRoutine routine = autoFactory.newRoutine(name);

        AutoTrajectory traj = routine.trajectory(trajName, 0);
        AutoTrajectory sprintTraj = routine.trajectory(trajName, 1);
        if (startingSide.mirrored) {
            traj = traj.mirrorY();
            sprintTraj = traj.mirrorY();
        }

        resetPoseInSim(routine, sprintTraj);

        routine.active()
                .onTrue(traj.cmd()
                        .alongWith(
                                intake.zeroWhileRunningCommand().andThen(intake.commandRunIntake(1.0)),
                                hood.zeroCommand().withTimeout(1.0)));
        traj.done()
                .onTrue(ShootingCommands.hubAimCommand(turret, shooter, hood)
                        .alongWith(FeedingCommands.feedCommand(turret, hood, spindexer), intake.jiggleCommand())
                        .withTimeout(sprint ? 5.5 : 10.0)
                        .andThen(sprintTraj.spawnCmd().onlyIf(() -> sprint)));

        return new AutoRoutineData(
                name,
                startingSide,
                sprintTraj.getInitialPose(),
                List.of(SweepPath.CLOSE_SWEEP),
                Stream.of(traj, sprintTraj)
                        .collect(
                                ArrayList::new,
                                (list, trajectory) -> list.addAll(
                                        List.of(trajectory.getRawTrajectory().getPoses())),
                                ArrayList::addAll),
                routine.cmd());
    }

    public AutoRoutineData doubleSweep(StartingSide startingSide, boolean hook, boolean sprint) {
        final String trajName = "DoubleSweep" + (hook ? "Hook" : "");

        String name = (startingSide == StartingSide.LEFT ? "Left" : "Right") + trajName + (sprint ? "Sprint" : "");
        AutoRoutine routine = autoFactory.newRoutine(name);

        AutoTrajectory firstSweep = routine.trajectory(trajName, 0);
        AutoTrajectory secondSweep = routine.trajectory(trajName, 1);
        AutoTrajectory sprintTraj = routine.trajectory(trajName, 2);
        if (startingSide.mirrored) {
            firstSweep = firstSweep.mirrorY();
            secondSweep = secondSweep.mirrorY();
            sprintTraj = sprintTraj.mirrorY();
        }

        resetPoseInSim(routine, firstSweep);

        routine.active()
                .onTrue(firstSweep
                        .cmd()
                        .alongWith(
                                intake.zeroWhileRunningCommand().andThen(intake.commandRunIntake(1.0)),
                                hood.zeroCommand().withTimeout(1.0)));
        firstSweep
                .done()
                .onTrue(ShootingCommands.hubAimCommand(turret, shooter, hood)
                        .alongWith(FeedingCommands.feedCommand(turret, hood, spindexer), intake.jiggleCommand())
                        .withTimeout(3.5)
                        .andThen(secondSweep.spawnCmd()));
        secondSweep.active().onTrue(intake.commandRunIntake(1.0));
        secondSweep
                .done()
                .onTrue(ShootingCommands.hubAimCommand(turret, shooter, hood)
                        .alongWith(FeedingCommands.feedCommand(turret, hood, spindexer), intake.jiggleCommand())
                        .withTimeout(sprint ? 5.5 : 10.0)
                        .andThen(sprintTraj.spawnCmd().onlyIf(() -> sprint)));

        return new AutoRoutineData(
                name,
                startingSide,
                firstSweep.getInitialPose(),
                List.of(SweepPath.CLOSE_SWEEP, hook ? SweepPath.CLOSE_HOOK : SweepPath.FAR_SWEEP),
                Stream.of(firstSweep, secondSweep, sprintTraj)
                        .collect(
                                ArrayList::new,
                                (list, trajectory) -> list.addAll(
                                        List.of(trajectory.getRawTrajectory().getPoses())),
                                ArrayList::addAll),
                routine.cmd());
    }
}
