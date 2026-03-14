package org.team1540.robot2026.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironmaple.simulation.SimulatedArena;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.RobotState;
import org.team1540.robot2026.commands.FeedingCommands;
import org.team1540.robot2026.commands.ShootingCommands;
import org.team1540.robot2026.subsystems.climber.Climber;
import org.team1540.robot2026.subsystems.drive.Drivetrain;
import org.team1540.robot2026.subsystems.hood.Hood;
import org.team1540.robot2026.subsystems.intake.Intake;
import org.team1540.robot2026.subsystems.shooter.Shooter;
import org.team1540.robot2026.subsystems.spindexer.Spindexer;
import org.team1540.robot2026.subsystems.turret.Turret;
import org.team1540.robot2026.util.AllianceFlipUtil;
import org.team1540.robot2026.util.auto.TrajectoryMirror;

public class Autos {
    public enum AutoSide {
        LEFT,
        RIGHT
    }

    private final RobotState robotState = RobotState.getInstance();
    private final AutoFactory autoFactory;

    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;
    private final Climber climber;

    public Autos(
            Drivetrain drivetrain,
            Intake intake,
            Spindexer spindexer,
            Turret turret,
            Hood hood,
            Shooter shooter,
            Climber climber) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.spindexer = spindexer;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        autoFactory = new AutoFactory(
                RobotState.getInstance()::getEstimatedPose,
                RobotState.getInstance()::resetPose,
                drivetrain::followTrajectory,
                true,
                drivetrain,
                (trajectory, starting) -> {
                    if (starting) {
                        RobotState.getInstance()
                                .setActiveTrajectory(
                                        (AllianceFlipUtil.shouldFlip() ? trajectory.flipped() : trajectory).getPoses());
                    } else {
                        RobotState.getInstance().clearActiveTrajectory();
                    }
                });
        this.climber = climber;
    }

    private void resetPoseInSim(AutoRoutine routine, AutoTrajectory startingTrajectory) {
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            routine.active().onTrue(Commands.runOnce(() -> {
                robotState.resetPose(startingTrajectory.getInitialPose().orElse(new Pose2d(3, 3, Rotation2d.kZero)));
                SimulatedArena.getInstance().resetFieldForAuto();
            }));
        }
    }

    public AutoRoutine testPath() {
        final String trajName = "TestAuto";

        AutoRoutine routine = autoFactory.newRoutine("TestAuto");
        AutoTrajectory traj = routine.trajectory(trajName);
        routine.active()
                .onTrue(traj.cmd()
                        .beforeStarting(Commands.runOnce(
                                () -> robotState.resetPose(traj.getInitialPose().orElse(Pose2d.kZero)))));

        return routine;
    }

    public AutoRoutine leftTrench1Sweep() {
        final String trajName = "LeftTrench1Sweep";

        AutoRoutine routine = autoFactory.newRoutine("LeftTrench1Sweep");
        AutoTrajectory traj = routine.trajectory(trajName);

        resetPoseInSim(routine, traj);

        routine.active()
                .onTrue(traj.cmd()
                        .alongWith(
                                intake.zeroWhileRunningCommand().andThen(intake.commandRunIntake(1.0)),
                                hood.zeroCommand().withTimeout(1.0)));
        traj.done()
                .onTrue(ShootingCommands.hubAimCommand(turret, shooter, hood)
                        .alongWith(FeedingCommands.feedCommand(turret, hood, spindexer), intake.jiggleCommand()));
        return routine;
    }

    public AutoRoutine leftTrench2Sweep(boolean shouldSprint) {
        final String trajName = "LeftTrench2Sweep";

        AutoRoutine routine = autoFactory.newRoutine("LeftTrench2Sweep");
        AutoTrajectory firstSweep = routine.trajectory(trajName, 0);
        AutoTrajectory secondSweep = routine.trajectory(trajName, 1);
        AutoTrajectory sprint = routine.trajectory(trajName, 2);

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
                        .withTimeout(shouldSprint ? 5.5 : 10.0)
                        .andThen(sprint.spawnCmd().onlyIf(() -> shouldSprint)));
        return routine;
    }

    public AutoRoutine leftTrench2SweepDepot() {
        final String trajName = "LeftTrench2SweepDepot";

        AutoRoutine routine = autoFactory.newRoutine("LeftTrench2SweepDepot");
        AutoTrajectory firstSweep = routine.trajectory(trajName, 0);
        AutoTrajectory secondSweep = routine.trajectory(trajName, 1);
        AutoTrajectory moveToDepot = routine.trajectory(trajName, 2);

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
                        .alongWith(
                                FeedingCommands.feedCommand(turret, hood, spindexer),
                                intake.jiggleCommand().asProxy(),
                                moveToDepot.spawnCmd()));
        moveToDepot.atTime("StartIntake").onTrue(intake.commandRunDepotIntake(1.0));
        moveToDepot.atTime("StopIntake").onTrue(intake.jiggleCommand());
        return routine;
    }

    public AutoRoutine rightTrench1Sweep() {
        final String trajName = "LeftTrench1Sweep";

        AutoRoutine routine = autoFactory.newRoutine("RightTrench1Sweep");
        AutoTrajectory traj = TrajectoryMirror.apply(routine.trajectory(trajName), routine);

        resetPoseInSim(routine, traj);

        routine.active()
                .onTrue(traj.cmd()
                        .alongWith(
                                intake.zeroWhileRunningCommand().andThen(intake.commandRunIntake(1.0)),
                                hood.zeroCommand().withTimeout(1.0)));
        traj.done()
                .onTrue(ShootingCommands.hubAimCommand(turret, shooter, hood)
                        .alongWith(FeedingCommands.feedCommand(turret, hood, spindexer), intake.jiggleCommand()));

        return routine;
    }

    public AutoRoutine rightTrench2Sweep(boolean shouldSprint) {
        final String trajName = "LeftTrench2Sweep";

        AutoRoutine routine = autoFactory.newRoutine("RightTrench2Sweep");
        AutoTrajectory firstSweep = TrajectoryMirror.apply(routine.trajectory(trajName, 0), routine);
        AutoTrajectory secondSweep = TrajectoryMirror.apply(routine.trajectory(trajName, 1), routine);
        AutoTrajectory sprint = TrajectoryMirror.apply(routine.trajectory(trajName, 2), routine);

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
                        .withTimeout(shouldSprint ? 5.5 : 10.0)
                        .andThen(sprint.spawnCmd().onlyIf(() -> shouldSprint)));
        return routine;
    }
}
