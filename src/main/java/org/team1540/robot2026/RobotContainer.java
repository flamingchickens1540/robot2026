package org.team1540.robot2026;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team1540.robot2026.commands.CharacterizationCommands;
import org.team1540.robot2026.commands.ShootingCommands;
import org.team1540.robot2026.subsystems.climber.Climber;
import org.team1540.robot2026.subsystems.climber.ClimberConstants;
import org.team1540.robot2026.subsystems.drive.Drivetrain;
import org.team1540.robot2026.subsystems.hood.Hood;
import org.team1540.robot2026.subsystems.hood.HoodConstants;
import org.team1540.robot2026.subsystems.intake.Intake;
import org.team1540.robot2026.subsystems.leds.CustomLEDPatterns;
import org.team1540.robot2026.subsystems.leds.LEDs;
import org.team1540.robot2026.subsystems.shooter.Shooter;
import org.team1540.robot2026.subsystems.spindexer.Spindexer;
import org.team1540.robot2026.subsystems.turret.Turret;
import org.team1540.robot2026.subsystems.vision.AprilTagVision;
import org.team1540.robot2026.util.hid.CommandEnvisionController;
import org.team1540.robot2026.util.hid.JoystickUtil;
import org.team1540.robot2026.util.MatchTriggers;
import org.team1540.robot2026.util.auto.LoggedAutoChooser;

public class RobotContainer {
    private final CommandEnvisionController driver = new CommandEnvisionController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    final Drivetrain drivetrain;
    final Intake intake;
    final Spindexer spindexer;
    final Shooter shooter;
    final Turret turret;
    final Hood hood;
    final Climber climber;
    final AprilTagVision vision;
    final LEDs leds = new LEDs();

    private final LoggedAutoChooser autoChooser = new LoggedAutoChooser("Auto Chooser");

    private final RobotState robotState = RobotState.getInstance();

    @AutoLogOutput(key = "ClimbMode")
    private boolean climbMode = false;

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
                vision = AprilTagVision.createReal();
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
                vision = AprilTagVision.createDummy();

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
                vision = AprilTagVision.createDummy();
            }
        }

        configureButtonBindings();
        configureLEDBindings();
        configureAutoRoutines();
        configureRobotModeTriggers();
        configurePeriodicCallbacks();
    }

    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(driver.getHID()));
        turret.setDefaultCommand(turret.commandToSetpoint(
                () -> robotState.getHubAimingParameters().turretAngle(),
                () -> robotState.getHubAimingParameters().turretVelocityRadPerSec(),
                true));

        driver.start().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));
        driver.back()
                .whileTrue(turret.zeroCommand()
                        .andThen(leds.viewFull.commandShowPattern(
                                CustomLEDPatterns.strobe(Color.kGreen, Seconds.of(0.5)))));

        driver.povRight().onTrue(Commands.runOnce(() -> climbMode = !climbMode));
        driver.leftTrigger()
                .whileTrue(Commands.either(
                        climber.runEnd(() -> climber.setVoltage(-0.67 * 12.0), climber::stop),
                        intake.commandRunIntake(1.0)
                                .alongWith(leds.viewFull.commandShowPattern(LEDPattern.solid(Color.kPurple))),
                        () -> climbMode));
        driver.leftOuterPaddle().whileTrue(intake.commandRunIntake(-0.67));
        driver.povLeft().onTrue(intake.commandToSetpoint(Intake.IntakeState.STOW));
        driver.rightOuterPaddle().onTrue(hood.setpointCommand(() -> HoodConstants.MIN_ANGLE));
        driver.rightBumper()
                .toggleOnTrue(ShootingCommands.hubAimCommand(turret, shooter, hood)
                        .alongWith(JoystickUtil.rumbleCommand(driver.getHID(), 1.0)));
        driver.leftBumper()
                .toggleOnTrue(ShootingCommands.shuffleAimCommand(turret, shooter, hood)
                        .alongWith(JoystickUtil.rumbleCommand(driver.getHID(), 1.0)));
        driver.rightTrigger()
                .whileTrue(Commands.either(
                        climber.runEnd(() -> climber.setVoltage(0.67 * 12.0), climber::stop),
                        spindexer.runCommand(() -> 1.0, () -> 1.0),
                        () -> climbMode));

        intake.setDefaultCommand(intake.run(() -> {
            double percent = JoystickUtil.smartDeadzone(copilot.getRightY(), 0.1);
            if (intake.getPivotPosition().getDegrees() <= 67
                    && percent < 0
                    && !copilot.rightBumper().getAsBoolean()) percent = 0;
            intake.setPivotVoltage(6 * percent);
        }));
        copilot.b()
                .toggleOnTrue(turret.run(
                        () -> turret.setVoltage(-JoystickUtil.smartDeadzone(copilot.getLeftX(), 0.1) * 0.5 * 12.0)));
        copilot.a().onTrue(hood.setpointCommand(() -> HoodConstants.MIN_ANGLE));
        copilot.start()
                .whileTrue(hood.zeroCommand()
                        .andThen(leds.viewFull.commandShowPattern(
                                CustomLEDPatterns.strobe(Color.kGreen, Seconds.of(0.5)))));
        copilot.back()
                .whileTrue(intake.zeroCommand()
                        .andThen(leds.viewFull.commandShowPattern(
                                CustomLEDPatterns.strobe(Color.kGreen, Seconds.of(0.5)))));
        copilot.leftBumper().whileTrue(spindexer.runCommand(() -> -0.67, () -> -0.67));
    }

    private void configureLEDBindings() {
        RobotModeTriggers.disabled()
                .whileTrue(leds.viewFull.commandShowPattern(CustomLEDPatterns.movingRainbow(Hertz.of(0.5))));
        RobotModeTriggers.teleop()
                .or(RobotModeTriggers.autonomous())
                .whileTrue(leds.viewFull.commandDefaultPattern(
                        () -> LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kOrange)
                                .scrollAtRelativeSpeed(Hertz.of(0.67))));
        new Trigger(() -> climbMode).whileTrue(leds.viewFull.commandShowPattern(LEDPattern.solid(Color.kCyan)));
        MatchTriggers.timeRemaining(30)
                .or(MatchTriggers.timeRemaining(15))
                .or(MatchTriggers.timeRemaining(10))
                .onTrue(leds.viewFull.commandShowPattern(CustomLEDPatterns.strobe(Color.kWhite, Seconds.of(1.5))));
    }

    private void configureAutoRoutines() {
        autoChooser.addCmd("Shoot Preload", () -> hood.zeroCommand()
                .alongWith(intake.zeroCommand())
                .withTimeout(1.0)
                .andThen(ShootingCommands.hubAimCommand(turret, shooter, hood)
                        .withDeadline(Commands.waitUntil(
                                        () -> turret.atSetpoint() && shooter.atSetpoint() && hood.isAtSetpoint())
                                .withTimeout(1.0)
                                .andThen(spindexer
                                        .runCommand(() -> 1.0, () -> 1.0)
                                        .withTimeout(5.0)))));
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
                            () -> climber.getVelocityMPS() / 2 * Math.PI * ClimberConstants.SPROCKET_RADIUS_M,
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
        addPeriodicCallback(robotState::periodic, "RobotStatePeriodic");
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
