package org.team1540.robot2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team1540.robot2026.commands.CharacterizationCommands;
import org.team1540.robot2026.subsystems.drive.Drivetrain;
import org.team1540.robot2026.subsystems.hood.Hood;
import org.team1540.robot2026.subsystems.hood.HoodConstants;
import org.team1540.robot2026.subsystems.shooter.Shooter;
import org.team1540.robot2026.subsystems.spindexer.Spindexer;
import org.team1540.robot2026.subsystems.turret.Turret;
import org.team1540.robot2026.subsystems.turret.TurretConstants;
import org.team1540.robot2026.util.JoystickUtil;
import org.team1540.robot2026.util.LoggedTunableNumber;
import org.team1540.robot2026.util.auto.LoggedAutoChooser;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    private final Drivetrain drivetrain;
    private final Hood hood;
    private final Shooter shooter;
    private final Spindexer spindexer;
    private final Turret turret;
    private final LoggedAutoChooser autoChooser = new LoggedAutoChooser("Auto Chooser");

    private final RobotState robotState = RobotState.getInstance();

    // TODO remove tuning setpoints
    private final LoggedTunableNumber shooterRPM = new LoggedTunableNumber("Shooter/SetpointRPM", 3000.0);
    private final LoggedTunableNumber hoodDegrees =
            new LoggedTunableNumber("Hood/SetpointDegrees", HoodConstants.MIN_ANGLE.getDegrees());
    private final LoggedTunableNumber turretDegrees =
            new LoggedTunableNumber("Turret/SetpointDegrees", TurretConstants.MIN_ANGLE.getDegrees());
    private final LoggedTunableNumber spindexerPercent = new LoggedTunableNumber("Spindexer/Percent", 0.5);
    private final LoggedTunableNumber feederPercent = new LoggedTunableNumber("Feeder/Percent", 0.5);

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                // Initialize physical hardware IOs
                drivetrain = Drivetrain.createReal();
                hood = Hood.createReal();
                shooter = Shooter.createReal();
                spindexer = Spindexer.createReal();
                turret = Turret.createDummy(); // TODO undummy
            }
            case SIM -> {
                // Initialize simulated hardware IOs
                drivetrain = Drivetrain.createSim();
                hood = Hood.createSim();
                shooter = Shooter.createSim();
                spindexer = Spindexer.createSim();
                turret = Turret.createSim();

                RobotState.getInstance().resetPose(new Pose2d(3.0, 3.0, Rotation2d.kZero));
            }
            default -> {
                // Initialize no-op hardware IOs for replay
                drivetrain = Drivetrain.createDummy();
                hood = Hood.createDummy();
                shooter = Shooter.createDummy();
                spindexer = Spindexer.createDummy();
                turret = Turret.createDummy();
            }
        }

        configureButtonBindings();
        configureAutoRoutines();
        configureRobotModeTriggers();
        configurePeriodicCallbacks();
    }

    private void configureButtonBindings() {
        // TODO retest DT
        //        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(driver.getHID()));
        //        driver.x().onTrue(drivetrain.runOnce(drivetrain::stopWithX));
        //        driver.start().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));
        turret.setDefaultCommand(
                turret.run(() -> turret.setVoltage(JoystickUtil.smartDeadzone(-driver.getLeftX(), 0.1) * 12.0))
                        .withName("TurretManual"));
        hood.setDefaultCommand(hood.run(() -> {
                    double joystickInput = JoystickUtil.smartDeadzone(-driver.getRightY(), 0.1);
                    hood.setVoltage(joystickInput * 12.0);
                })
                .withName("HoodManual"));

        driver.start().whileTrue(hood.zeroCommand());
        driver.back().onTrue(turret.runOnce(() -> turret.resetPosition(TurretConstants.MIN_ANGLE)));

        driver.a().toggleOnTrue(shooter.runVelocityCommand(shooterRPM));
        driver.b().toggleOnTrue(hood.setpointCommand(() -> Rotation2d.fromDegrees(hoodDegrees.get())));
        driver.y().toggleOnTrue(turret.commandToSetpoint(() -> Rotation2d.fromDegrees(turretDegrees.get())));
        driver.x()
                .onTrue(Commands.parallel(
                                shooter.runOnce(shooter::stop), hood.runOnce(hood::stop), turret.runOnce(turret::stop))
                        .withName("StopAll"));
        driver.rightTrigger().whileTrue(spindexer.runCommand(spindexerPercent, feederPercent));
    }

    private void configureAutoRoutines() {
        // Characterization routines
        if (Constants.isTuningMode()) {
            autoChooser.addCmd(
                    "Drive FF Characterization",
                    () -> CharacterizationCommands.feedforward(
                            drivetrain::runCharacterization, drivetrain::getFFCharacterizationVelocity, drivetrain));
            autoChooser.addCmd(
                    "Drive Wheel Radius Characterization",
                    () -> CharacterizationCommands.wheelRadius(
                            input -> drivetrain.runVelocity(new ChassisSpeeds(0.0, 0.0, input)),
                            () -> RobotState.getInstance().getRobotHeading().getRadians(),
                            drivetrain::getWheelRadiusCharacterizationPositions,
                            drivetrain));
            autoChooser.addCmd(
                    "Shooter FF Characterization",
                    () -> CharacterizationCommands.feedforward(
                            shooter::setVoltage, () -> shooter.getVelocityRPM() / 60.0, shooter));
            autoChooser.addCmd(
                    "Hood FF Characterization",
                    () -> CharacterizationCommands.feedforward(hood::setVoltage, hood::getVelocityRPS, hood));
            autoChooser.addCmd(
                    "Turret FF Characterization",
                    () -> CharacterizationCommands.feedforward(turret::setVoltage, turret::getVelocityRPS, turret));
        }
    }

    private void configureRobotModeTriggers() {
        RobotModeTriggers.teleop()
                .and(DriverStation::isFMSAttached)
                .onTrue(drivetrain.runOnce(drivetrain::zeroFieldOrientation));
    }

    private void configurePeriodicCallbacks() {
        addPeriodicCallback(autoChooser::update, "AutoChooserUpdate");
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            addPeriodicCallback(SimState.getInstance()::update, "SimulationUpdate");
        }
    }

    private void addPeriodicCallback(Runnable callback, String name) {
        CommandScheduler.getInstance()
                .schedule(Commands.run(callback).withName(name).ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.selectedCommandScheduler();
    }
}
