package org.team1540.robot2026;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team1540.robot2026.autos.Autos;
import org.team1540.robot2026.commands.CharacterizationCommands;
import org.team1540.robot2026.commands.FeedingCommands;
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
import org.team1540.robot2026.util.HubShiftUtil;
import org.team1540.robot2026.util.MatchTriggers;
import org.team1540.robot2026.util.auto.LoggedAutoChooser;
import org.team1540.robot2026.util.hid.CommandEnvisionController;
import org.team1540.robot2026.util.hid.JoystickUtil;

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

    private final Autos autos;

    @AutoLogOutput(key = "Turret/LockedMode")
    private boolean turretLockedMode = false;

    private final Alert turretLockedAlert = new Alert("Turret is in locked mode!", Alert.AlertType.kError);
    private final Alert turretManualAlert = new Alert("Turret is under manual control", Alert.AlertType.kWarning);
    private final Alert intakeManualAlert = new Alert("Intake is under manual control", Alert.AlertType.kWarning);

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
                climber = Climber.createDummy();
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

        autos = new Autos(drivetrain, intake, spindexer, turret, hood, shooter, climber);

        configureButtonBindings();
        configureAutoRoutines();
        configureRobotModeTriggers();
        configurePeriodicCallbacks();
    }

    private void configureButtonBindings() {
        Trigger manualFeedOverride = driver.leftBumper().or(() -> turretLockedMode);
        Command intakeCmd = intake.commandRunIntake(1.0).withName("IntakeCommand");
        Command shootCmd = Commands.either(
                        ShootingCommands.shooterAimTurretLockedCommand(driver.getHID(), drivetrain, shooter, hood)
                                .withName("ShooterAimTurretLockedCommand")
                                .asProxy(),
                        ShootingCommands.shooterAimCommand(turret, shooter, hood)
                                .withName("ShooterAimCommand")
                                .asProxy(),
                        () -> turretLockedMode)
                .deadlineFor(
                        FeedingCommands.feedCommand(turret, hood, spindexer, manualFeedOverride),
                        intake.jiggleCommand()
                                .asProxy()
                                .unless(intakeCmd::isScheduled)
                                .repeatedly(),
                        JoystickUtil.rumbleCommand(driver.getHID(), 1.0))
                .withName("ShootCommand");

        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(
                driver.getHID(),
                () -> shootCmd.isScheduled() && robotState.getTargetingMode() == RobotState.TargetingMode.HUB));
        turret.setDefaultCommand(turret.commandToSetpoint(
                () -> robotState.getAimingParameters().turretAngle(),
                () -> robotState.getAimingParameters().turretVelocityRadPerSec(),
                true));
        driver.x().onTrue(drivetrain.runOnce(drivetrain::stopWithX).withName("DriveXMode"));
        driver.start()
                .onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual).withName("ManualDriveZero"));

        // Shoot/intake controls
        driver.leftTrigger().toggleOnTrue(intakeCmd);
        driver.rightTrigger().toggleOnTrue(shootCmd);

        // Climb controls
        driver.leftSideButton()
                .whileTrue(climber.runEnd(() -> climber.setVoltage(-0.67 * 12.0), climber::stop)
                        .withName("ClimbDownCommand"));
        driver.rightSideButton()
                .whileTrue(climber.runEnd(() -> climber.setVoltage(0.67 * 12.0), climber::stop)
                        .withName("ClimbUpCommand"));

        // Misc controls
        driver.leftInnerPaddle().whileTrue(intake.commandRunIntake(-0.67).withName("OuttakeCommand"));
        driver.leftOuterPaddle()
                .onTrue(intake.commandToSetpoint(Intake.IntakeState.STOW).withName("StowIntakeCommand"));
        driver.rightOuterPaddle()
                .onTrue(hood.setpointCommand(() -> HoodConstants.MIN_ANGLE).withName("HoodDownCommand"));
        driver.leftBumper().onTrue(turret.run(turret::stop).withName("TurretStopCommand"));
        driver.back()
                .onTrue(turret.zeroCommand()
                        .asProxy()
                        .andThen(leds.viewFull
                                .commandShowPattern(CustomLEDPatterns.strobe(Color.kGreen))
                                .withTimeout(0.5))
                        .ignoringDisable(true));

        // Copilot controls
        copilot.povUp()
                .onTrue(Commands.runOnce(() -> RobotState.getInstance().incrementShooterRPMOffset(20))
                        .ignoringDisable(true));
        copilot.povDown()
                .onTrue(Commands.runOnce(() -> RobotState.getInstance().incrementShooterRPMOffset(-20))
                        .ignoringDisable(true));

        copilot.back()
                .onTrue(turret.zeroCommand()
                        .asProxy()
                        .andThen(leds.viewFull
                                .commandShowPattern(CustomLEDPatterns.strobe(Color.kGreen))
                                .withTimeout(0.5))
                        .ignoringDisable(true));
        copilot.a().onTrue(hood.setpointCommand(() -> HoodConstants.MIN_ANGLE).withName("HoodDownCommand"));
        copilot.b()
                .and(() -> !turretLockedMode)
                .toggleOnTrue(turret.run(() -> {
                            turret.setVoltage(-JoystickUtil.smartDeadzone(copilot.getLeftX(), 0.1) * 0.5 * 12.0);
                            turretManualAlert.set(true);
                        })
                        .finallyDo(() -> {
                            turret.setVoltage(0.0);
                            turretManualAlert.set(false);
                        })
                        .withName("TurretManualControl"));
        copilot.x()
                .toggleOnTrue(intake.run(() -> {
                            intake.setPivotVoltage(JoystickUtil.smartDeadzone(copilot.getRightY(), 0.1) * 0.5 * 12.0);
                            intakeManualAlert.set(true);
                        })
                        .finallyDo(() -> {
                            intake.setPivotVoltage(0.0);
                            intakeManualAlert.set(false);
                        })
                        .withName("IntakeManualControl"));
        copilot.start()
                .whileTrue(hood.zeroCommand()
                        .andThen(leds.viewFull
                                .commandShowPattern(CustomLEDPatterns.strobe(Color.kGreen))
                                .withTimeout(0.5))
                        .withName("HoodZeroCommand"));

        copilot.leftTrigger()
                .whileTrue(intake.zeroCommand()
                        .andThen(leds.viewFull
                                .commandShowPattern(CustomLEDPatterns.strobe(Color.kGreen))
                                .withTimeout(0.5))
                        .withName("IntakeZeroCommand"));
        copilot.leftBumper()
                .whileTrue(spindexer.runCommand(() -> -0.67, () -> -0.67).withName("SpindexerReverseCommand"));
        copilot.rightStick()
                .toggleOnTrue(turret.run(turret::stop)
                        .alongWith(Commands.runOnce(() -> {
                            turretLockedMode = true;
                            turretLockedAlert.set(true);
                        }))
                        .finallyDo(() -> {
                            turretLockedMode = false;
                            turretLockedAlert.set(false);
                        }));
        copilot.rightTrigger()
                .whileTrue(ShootingCommands.closeShotCommand(shooter, hood)
                        .alongWith(
                                FeedingCommands.feedCommand(turret, hood, spindexer, manualFeedOverride),
                                intake.jiggleCommand())
                        .withName("CloseShotCommand"));
        copilot.rightBumper()
                .whileTrue(ShootingCommands.trenchShotCommand(shooter, hood)
                        .alongWith(
                                FeedingCommands.feedCommand(turret, hood, spindexer, manualFeedOverride),
                                intake.jiggleCommand())
                        .withName("trenchShotCommand"));

        // Shooter tuning bindings
        if (Constants.isTuningMode()) {
            copilot.y()
                    .toggleOnTrue(ShootingCommands.tuneShooterCommand(turret, shooter, hood)
                            .withName("TuneShooterCommand"));
            copilot.povUp()
                    .whileTrue(
                            FeedingCommands.feedCommand(turret, hood, spindexer).alongWith(intake.jiggleCommand()));
        }

        // LED bindings
        Supplier<LEDPattern> disabledPattern = () -> {
            if (!DriverStation.isDSAttached())
                return LEDPattern.solid(Color.kWhite).breathe(Seconds.of(1.5));
            else if (!turret.isZeroed()) return CustomLEDPatterns.strobe(Color.kRed);
            return CustomLEDPatterns.movingRainbow(Hertz.of(0.5));
        };
        Supplier<LEDPattern> teleopPattern = () -> {
            if (turretLockedMode) return CustomLEDPatterns.strobe(Color.kRed);
            else if (intakeCmd.isScheduled()) {
                if (!shootCmd.isScheduled()) return LEDPattern.solid(Color.kPurple);
                else return LEDPattern.solid(Color.kPurple).blink(Seconds.of(0.25));
            } else if (shootCmd.isScheduled()) {
                return LEDPattern.solid(LEDs.getAllianceColor()).blink(Seconds.of(0.25));
            } else return LEDPattern.solid(LEDs.getAllianceColor());
        };
        Supplier<LEDPattern> autoPattern =
                () -> LEDPattern.solid(LEDs.getAllianceColor()).blink(Seconds.of(0.25));

        leds.viewFull.setDefaultPattern(disabledPattern);
        RobotModeTriggers.disabled().whileTrue(leds.viewFull.commandDefaultPattern(disabledPattern));
        RobotModeTriggers.teleop().whileTrue(leds.viewFull.commandDefaultPattern(teleopPattern));
        RobotModeTriggers.autonomous().whileTrue(leds.viewFull.commandDefaultPattern(autoPattern));

        new Trigger(() -> !FeedingCommands.shouldFeed(turret, hood, manualFeedOverride) && shootCmd.isScheduled())
                .whileTrue(leds.viewFull.commandShowPattern(CustomLEDPatterns.strobe(Color.kOrange)));
        MatchTriggers.timeRemaining(30)
                .onTrue(leds.viewFull
                        .commandShowPattern(CustomLEDPatterns.strobe(Color.kWhite))
                        .withTimeout(1.0));
    }

    private void configureAutoRoutines() {
        autoChooser.addCmd("Shoot Preload", () -> hood.zeroCommand()
                .alongWith(intake.zeroCommand())
                .withTimeout(1.0)
                .andThen(ShootingCommands.hubAimCommand(turret, shooter, hood)
                        .withDeadline(Commands.waitUntil(
                                        () -> turret.atSetpoint() && shooter.atSetpoint() && hood.atSetpoint())
                                .withTimeout(1.0)
                                .andThen(spindexer
                                        .runCommand(() -> 1.0, () -> 1.0)
                                        .withTimeout(5.0)))));
        autoChooser.addRoutine("Left Trench 1 Sweep", autos::leftTrench1Sweep);
        autoChooser.addRoutine("Left Trench 2 Sweep", () -> autos.leftTrench2Sweep(false));
        autoChooser.addRoutine("Left Trench 2 Sweep Sprint", () -> autos.leftTrench2Sweep(true));
        autoChooser.addRoutine("Left Trench 2 Sweep Depot", autos::leftTrench2SweepDepot);
        autoChooser.addRoutine("Right Trench 1 Sweep", autos::rightTrench1Sweep);
        autoChooser.addRoutine("Right Trench 2 Sweep", () -> autos.rightTrench2Sweep(false));
        autoChooser.addRoutine("Right Trench 2 Sweep Sprint", () -> autos.rightTrench2Sweep(true));
        autoChooser.addRoutine("Left Trench Far 2 Sweep Sprint", () -> autos.leftTrenchFar2SweepSprint(true));

        // Characterization routines
        if (Constants.isTuningMode()) {
            autoChooser.addRoutine("Test", autos::testPath);
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
                .onTrue(drivetrain.runOnce(drivetrain::zeroFieldOrientation).withName("DriveOrientToOdometry"));

        // Reset hub shift timer at the start of each mode
        RobotModeTriggers.teleop()
                .onTrue(Commands.runOnce(HubShiftUtil::initialize).withName("HubShiftInit"));
        RobotModeTriggers.autonomous()
                .onTrue(Commands.runOnce(HubShiftUtil::initialize).withName("HubShiftInit"));
        RobotModeTriggers.disabled()
                .onTrue(Commands.runOnce(HubShiftUtil::initialize)
                        .ignoringDisable(true)
                        .withName("HubShiftInit"));
    }

    private void configurePeriodicCallbacks() {
        addPeriodicCallback(autoChooser::update, "AutoChooserUpdate");
        addPeriodicCallback(robotState::periodic, "RobotStatePeriodic");
        addPeriodicCallback(HubShiftUtil::periodic, "HubShiftPeriodic");
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
