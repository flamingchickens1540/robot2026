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
import org.team1540.robot2026.commands.ShootingCommands;
import org.team1540.robot2026.subsystems.drive.Drivetrain;
import org.team1540.robot2026.subsystems.hood.Hood;
import org.team1540.robot2026.subsystems.intake.Intake;
import org.team1540.robot2026.subsystems.shooter.Shooter;
import org.team1540.robot2026.subsystems.spindexer.Spindexer;
import org.team1540.robot2026.subsystems.turret.Turret;
import org.team1540.robot2026.util.AllianceFlipUtil;

public class Autos {
    private final RobotState robotState = RobotState.getInstance();
    private final AutoFactory autoFactory;

    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;

    public Autos(Drivetrain drivetrain, Intake intake, Spindexer spindexer, Turret turret, Hood hood, Shooter shooter) {
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
        autoFactory.bind("StartIntake", intake.commandRunIntake(1.0));
        autoFactory.bind("StopIntake", intake.commandRunIntake(0.0));
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

    public AutoRoutine depotTNPH() {
        final String trajName = "DepotTNPH";

        AutoRoutine routine = autoFactory.newRoutine("DepotTNPH");
        AutoTrajectory traj = routine.trajectory(trajName);

        resetPoseInSim(routine, traj);

        routine.active()
                .onTrue(traj.cmd().alongWith(hood.zeroCommand().withTimeout(1.0).asProxy()));
        traj.done()
                .onTrue(ShootingCommands.hubAimCommand(turret, shooter, hood)
                        .alongWith(spindexer.runCommand(() -> 1.0, () -> 1.0), intake.jiggleCommand())
                        .withTimeout(5.0));
        return routine;
    }

    public AutoRoutine depotTNPHTNH() {
        final String trajName = "DepotTNPHTNH";

        AutoRoutine routine = autoFactory.newRoutine("DepotTNPHTNH");
        AutoTrajectory firstSweep = routine.trajectory(trajName, 0);
        AutoTrajectory secondSweep = routine.trajectory(trajName, 1);

        resetPoseInSim(routine, firstSweep);

        routine.active()
                .onTrue(firstSweep
                        .cmd()
                        .alongWith(
                                intake.commandRunIntake(1.0),
                                hood.zeroCommand().withTimeout(1.0).asProxy()));
        firstSweep
                .done()
                .onTrue(ShootingCommands.hubAimCommand(turret, shooter, hood)
                        .alongWith(spindexer.runCommand(() -> 1.0, () -> 1.0), intake.jiggleCommand())
                        .withTimeout(3.0)
                        .andThen(secondSweep.spawnCmd()));
        secondSweep.active().onTrue(intake.commandRunIntake(1.0));
        secondSweep
                .done()
                .onTrue(ShootingCommands.hubAimCommand(turret, shooter, hood)
                        .alongWith(spindexer.runCommand(() -> 1.0, () -> 1.0), intake.jiggleCommand())
                        .withTimeout(3.0));
        return routine;
    }

    public AutoRoutine depotTNPHTNHDH() {
        final String trajName = "DepotTNPHTNHDH";

        AutoRoutine routine = autoFactory.newRoutine("DepotTNPHTNH");
        AutoTrajectory firstSweep = routine.trajectory(trajName, 0);
        AutoTrajectory secondSweep = routine.trajectory(trajName, 1);

        resetPoseInSim(routine, firstSweep);

        routine.active()
                .onTrue(firstSweep
                        .cmd()
                        .alongWith(
                                intake.commandRunIntake(1.0),
                                hood.zeroCommand().withTimeout(1.0).asProxy()));
        firstSweep
                .done()
                .onTrue(ShootingCommands.hubAimCommand(turret, shooter, hood)
                        .alongWith(spindexer.runCommand(() -> 1.0, () -> 1.0), intake.jiggleCommand())
                        .withTimeout(3.0)
                        .andThen(secondSweep.spawnCmd()));
        secondSweep.active().onTrue(intake.commandRunIntake(1.0));
        secondSweep
                .done()
                .onTrue(ShootingCommands.hubAimCommand(turret, shooter, hood)
                        .alongWith(spindexer.runCommand(() -> 1.0, () -> 1.0), intake.jiggleCommand())
                        .withTimeout(3.0));
        return routine;
    }
}
