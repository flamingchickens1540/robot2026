package org.team1540.robot2026.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironmaple.simulation.SimulatedArena;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.RobotState;
import org.team1540.robot2026.commands.ShootingCommands;
import org.team1540.robot2026.subsystems.climber.Climber;
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
    private final Climber climber;

    public Autos(Drivetrain drivetrain, Intake intake, Spindexer spindexer, Turret turret, Hood hood, Shooter shooter, Climber climber) {
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

    public AutoRoutine depotDPC(){
        final String trajName = "DepotDPC";

        AutoRoutine routine =  autoFactory.newRoutine("DepotDPC");
        AutoTrajectory hub2Depot = routine.trajectory(trajName, 0);
        AutoTrajectory rest = routine.trajectory(trajName, 1);

        Command intakeCmd = intake.commandRunIntake(1.0)
                .withName("IntakeCommand");
        Command feedShooterCmd = spindexer.runCommand(() -> 1.0, () -> 1.0);
        hub2Depot
                .atTime("DeployIntake")
                .onTrue(intake.commandRunIntake(0.5).alongWith(feedShooterCmd
                        .alongWith(intake.jiggleCommand()
                                .asProxy()
                                .unless(intakeCmd::isScheduled)
                                .repeatedly()
                        )));
        rest
                .atTime("Climb")
                .onTrue(climber.runEnd(() -> climber.setVoltage(-0.67 * 12.0), climber::stop).withTimeout(2).andThen(climber.runEnd(() -> climber.setVoltage(-0.67 * 12.0), climber::stop)));

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
