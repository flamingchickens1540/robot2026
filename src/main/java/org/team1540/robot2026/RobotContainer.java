package org.team1540.robot2026;

import static org.team1540.robot2026.subsystems.climber.ClimberConstants.SPROCKET_RADIUS_M;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team1540.robot2026.commands.CharacterizationCommands;
import org.team1540.robot2026.subsystems.climber.Climber;
import org.team1540.robot2026.subsystems.drive.Drivetrain;
import org.team1540.robot2026.subsystems.hood.Hood;
import org.team1540.robot2026.subsystems.intake.Intake;
import org.team1540.robot2026.subsystems.shooter.Shooter;
import org.team1540.robot2026.subsystems.spindexer.Spindexer;
import org.team1540.robot2026.subsystems.turret.Turret;
import org.team1540.robot2026.util.JoystickUtil;
import org.team1540.robot2026.util.LoggedTunableNumber;
import org.team1540.robot2026.util.auto.LoggedAutoChooser;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    final Drivetrain drivetrain;
    final Intake intake;
    final Spindexer spindexer;
    final Shooter shooter;
    final Turret turret;
    final Hood hood;
    final Climber climber;

    private final LoggedAutoChooser autoChooser = new LoggedAutoChooser("Auto Chooser");

    private final RobotState robotState = RobotState.getInstance();

    @AutoLogOutput(key = "ClimbMode")
    private boolean climbMode = false;

    // TODO remove tunables
    private final LoggedTunableNumber hoodDegrees = new LoggedTunableNumber("Hood/SetpointDegrees", 30.0);
    private final LoggedTunableNumber shooterRPM = new LoggedTunableNumber("Shooter/SetpointRPM", 1000);
    private final LoggedTunableNumber spindexerPercent = new LoggedTunableNumber("Spindexer/SpinPercent", 0.75);
    private final LoggedTunableNumber feederPercent = new LoggedTunableNumber("Spindexer/FeedPercent", 0.75);
    private final LoggedTunableNumber intakePercent = new LoggedTunableNumber("Intake/Percent", 0.75);

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                // Initialize physical hardware IOs
                drivetrain = Drivetrain.createReal();
                intake = Intake.createReal();
                spindexer = Spindexer.createReal();
                shooter = Shooter.createReal();
                turret = Turret.createReal();
                hood = Hood.createReal();
                climber = Climber.createReal();
            }
            case SIM -> {
                // Initialize simulated hardware IOs
                drivetrain = Drivetrain.createSim();
                intake = Intake.createSim();
                spindexer = Spindexer.createSim();
                shooter = Shooter.createSim();
                turret = Turret.createSim();
                hood = Hood.createSim();
                climber = Climber.createSim();

                RobotState.getInstance().resetPose(new Pose2d(3.0, 3.0, Rotation2d.kZero));
            }
            default -> {
                // Initialize no-op hardware IOs for replay
                drivetrain = Drivetrain.createDummy();
                intake = Intake.createDummy();
                spindexer = Spindexer.createDummy();
                shooter = Shooter.createDummy();
                turret = Turret.createDummy();
                hood = Hood.createDummy();
                climber = Climber.createDummy();
            }
        }

        configureButtonBindings();
        configureAutoRoutines();
        configureRobotModeTriggers();
        configurePeriodicCallbacks();
    }

    private void configureButtonBindings() {
        // Driving commands
        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(driver.getHID()));
        driver.start().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));

        intake.setDefaultCommand(intake.run(() -> {
            intake.setPivotVoltage(6 * JoystickUtil.smartDeadzone(copilot.getRightY(), 0.1));
            if (copilot.leftTrigger().getAsBoolean()) intake.setRollerVoltage(12.0 * intakePercent.get());
            else intake.setRollerVoltage(0);
        }));
        copilot.b()
                .toggleOnTrue(turret.run(
                        () -> turret.setVoltage(-JoystickUtil.smartDeadzone(copilot.getLeftX(), 0.1) * 0.5 * 12.0)));
        copilot.a()
                .toggleOnTrue(hood.setpointCommand(() -> Rotation2d.fromDegrees(hoodDegrees.get()))
                        .alongWith(shooter.commandVelocity(shooterRPM)));
        copilot.rightTrigger().whileTrue(spindexer.runCommand(spindexerPercent, feederPercent));
        copilot.start().whileTrue(hood.zeroCommand());

        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(driver.getHID()));

        driver.leftTrigger()
                .whileTrue(Commands.either(
                        intake.commandRunIntake(0.67),
                        climber.run(() -> climber.setVoltage(-0.67 * 12.0)),
                        () -> climbMode));
        driver.leftStick().whileTrue(intake.commandRunIntake(-0.67));
        driver.povLeft().onTrue(intake.commandToSetpoint(Intake.IntakeState.STOW));
        driver.rightStick().onTrue(hood.setpointCommand(() -> Rotation2d.kZero));
        //        driver.povRight().onTrue(Commands.parallel(ClimbAutoAlign, Commands.run(()->climbMode=!climbMode)));
        //        driver.rightTrigger().whileTrue(Commands.either(
        //                ShootSequence,
        //                climber.run(() -> climber.setVoltage(0.67 * 12.0)),
        //                ()->climbMode));

        driver.start().whileTrue(turret.zeroCommand());
        driver.back().onTrue(Commands.run(drivetrain::zeroFieldOrientation));
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
                    "Intake FF Characterization",
                    () -> CharacterizationCommands.feedforward(
                            intake::setPivotVoltage, intake::getPivotVelocityRPS, intake));
            autoChooser.addCmd(
                    "Shooter FF Characterization",
                    () -> CharacterizationCommands.feedforward(
                            shooter::setVoltage, () -> shooter.getVelocityRPM() / 60, shooter));
            autoChooser.addCmd(
                    "Climber FF Characterization",
                    () -> CharacterizationCommands.feedforward(
                            climber::setVoltage,
                            () -> climber.getVelocityMPS() / 2 * Math.PI * SPROCKET_RADIUS_M,
                            climber));
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
        return autoChooser.selectedCommand();
    }
}
