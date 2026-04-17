package org.team1540.robot2026;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team1540.robot2026.autos.AutoConfigurator;
import org.team1540.robot2026.autos.AutoConfigurator.StartingSide;
import org.team1540.robot2026.autos.AutoPresets;
import org.team1540.robot2026.autos.AutoSelector;
import org.team1540.robot2026.commands.CharacterizationCommands;
import org.team1540.robot2026.commands.FeedingCommands;
import org.team1540.robot2026.commands.ShootingCommands;
import org.team1540.robot2026.controls.CopilotControls;
import org.team1540.robot2026.controls.DriverControls;
import org.team1540.robot2026.subsystems.climber.Climber;
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
import org.team1540.robot2026.util.AllianceFlipUtil;
import org.team1540.robot2026.util.HubShiftUtil;
import org.team1540.robot2026.util.MatchTriggers;
import org.team1540.robot2026.util.hid.JoystickUtil;
import org.team1540.robot2026.util.logging.LoggedTracer;

public class RobotContainer {
    private final DriverControls driver = DriverControls.EnvisionController;
    private final CopilotControls copilot = CopilotControls.XboxController;

    final Drivetrain drivetrain;
    final Intake intake;
    final Spindexer spindexer;
    final Shooter shooter;
    final Turret turret;
    final Hood hood;
    final Climber climber;
    final AprilTagVision vision;
    final LEDs leds = new LEDs();

    private final RobotState robotState = RobotState.getInstance();

    final AutoFactory autoFactory;
    private final AutoPresets autoPresets;
    private final AutoConfigurator autoConfigurator;
    private final AutoSelector autoSelector;

    @AutoLogOutput(key = "Turret/LockedMode")
    private boolean turretLockedMode = false;

    private final Alert turretLockedAlert = new Alert("Turret is in locked mode!", Alert.AlertType.kError);
    private final Alert turretManualAlert = new Alert("Turret is under manual control", Alert.AlertType.kWarning);
    private final Alert intakeManualAlert = new Alert("Intake is under manual control", Alert.AlertType.kWarning);

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
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
                climber = Climber.createDummy();
                vision = AprilTagVision.createSim();
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
        autoPresets = new AutoPresets(autoFactory, drivetrain, intake, spindexer, turret, hood, shooter);
        autoConfigurator = new AutoConfigurator(autoFactory, drivetrain, intake, spindexer, turret, hood, shooter);
        autoSelector = new AutoSelector(autoConfigurator);

        configureButtonBindings();
        configureAutoRoutines();
        configureRobotModeTriggers();
        configurePeriodicCallbacks();
    }

    private void configureButtonBindings() {
        Trigger manualFeedOverride = driver.forceShoot.or(() -> turretLockedMode);
        Command intakeCmd = intake.commandRunIntakeAutoReverse();
        Command shootCmd = Commands.either(
                        ShootingCommands.shooterAimTurretLockedCommand(
                                        driver.driveX, driver.driveY, driver.driveRotation, drivetrain, shooter, hood)
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
                        JoystickUtil.rumbleCommand(driver.hid, 1.0))
                .withName("ShootCommand");

        /* Driver controls */

        // Drivetrain controls
        Trigger rateLimitActive = new Trigger(
                () -> shootCmd.isScheduled() && robotState.getTargetingMode() == RobotState.TargetingMode.HUB);
        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(driver.driveX, driver.driveY, driver.driveRotation, rateLimitActive));
        driver.driveXMode.whileTrue(drivetrain.run(drivetrain::stopWithX).withName("DriveXMode"));
        driver.pointMode
                .and(intakeCmd::isScheduled)
                .whileTrue(drivetrain.teleopDrivePointCommand(
                        driver.driveX, driver.driveY, driver.driveRotation, rateLimitActive));
        driver.zeroDriveOrientation.onTrue(
                Commands.runOnce(drivetrain::zeroFieldOrientationManual).withName("ManualDriveZero"));

        // Shoot/intake controls
        driver.intake.toggleOnTrue(intakeCmd);
        driver.shoot.toggleOnTrue(shootCmd);

        // Misc controls
        driver.outtake.whileTrue(intake.commandRunIntake(-0.67)
                .alongWith(spindexer.runCommand(() -> 0.0, () -> -0.67, () -> -0.67))
                .withName("OuttakeCommand"));
        driver.stowIntake.onTrue(
                intake.commandToSetpoint(Intake.IntakeState.STOW).withName("StowIntakeCommand"));
        driver.stowHood.onTrue(
                hood.setpointCommand(() -> HoodConstants.MIN_ANGLE).withName("StowHoodCommand"));
        driver.stopTurret.onTrue(turret.run(turret::stop).withName("TurretStopCommand"));
        driver.zeroTurret.onTrue(turret.zeroCommand()
                .asProxy()
                .andThen(leds.viewFull
                        .commandShowPattern(CustomLEDPatterns.strobe(Color.kGreen))
                        .withTimeout(0.5))
                .ignoringDisable(true));

        /* Copilot controls */

        // Zeroing controls
        copilot.zeroTurret.onTrue(turret.zeroCommand()
                .asProxy()
                .andThen(leds.viewFull
                        .commandShowPattern(CustomLEDPatterns.strobe(Color.kGreen))
                        .withTimeout(0.5))
                .ignoringDisable(true));
        copilot.zeroHood.whileTrue(hood.zeroCommand()
                .andThen(leds.viewFull
                        .commandShowPattern(CustomLEDPatterns.strobe(Color.kGreen))
                        .withTimeout(0.5))
                .withName("HoodZeroCommand"));
        copilot.zeroIntake.whileTrue(intake.zeroCommand()
                .andThen(leds.viewFull
                        .commandShowPattern(CustomLEDPatterns.strobe(Color.kGreen))
                        .withTimeout(0.5))
                .withName("IntakeZeroCommand"));

        // Manual mechanism controls
        copilot.manualTurret
                .and(() -> !turretLockedMode)
                .toggleOnTrue(turret.run(() -> {
                            turret.setVoltage(-JoystickUtil.smartDeadzone(copilot.manualTurretInput.getAsDouble(), 0.1)
                                    * 0.5
                                    * 12.0);
                            turretManualAlert.set(true);
                        })
                        .finallyDo(() -> {
                            turret.setVoltage(0.0);
                            turretManualAlert.set(false);
                        })
                        .withName("TurretManualControl"));
        copilot.manualIntake.toggleOnTrue(intake.run(() -> {
                    intake.setPivotVoltage(
                            JoystickUtil.smartDeadzone(copilot.manualTurretInput.getAsDouble(), 0.1) * 0.5 * 12.0);
                    intakeManualAlert.set(true);
                })
                .finallyDo(() -> {
                    intake.setPivotVoltage(0.0);
                    intakeManualAlert.set(false);
                })
                .withName("IntakeManualControl"));

        // Misc mechanism controls
        copilot.reverseSpindexer.whileTrue(
                spindexer.runCommand(() -> -0.67, () -> -0.67, () -> 0.67).withName("SpindexerReverseCommand"));
        copilot.stowHood.onTrue(
                hood.setpointCommand(() -> HoodConstants.MIN_ANGLE).withName("StowHoodCommand"));
        copilot.lockTurret.toggleOnTrue(turret.run(turret::stop)
                .alongWith(Commands.runOnce(() -> {
                    turretLockedMode = true;
                    turretLockedAlert.set(true);
                }))
                .finallyDo(() -> {
                    turretLockedMode = false;
                    turretLockedAlert.set(false);
                }));

        // Pretuned shots
        copilot.closeShot.whileTrue(ShootingCommands.closeShotCommand(shooter, hood)
                .alongWith(
                        FeedingCommands.feedCommand(turret, hood, spindexer, manualFeedOverride),
                        intake.jiggleCommand())
                .withName("CloseShotCommand"));
        copilot.trenchShot.whileTrue(ShootingCommands.trenchShotCommand(shooter, hood)
                .alongWith(
                        FeedingCommands.feedCommand(turret, hood, spindexer, manualFeedOverride),
                        intake.jiggleCommand())
                .withName("trenchShotCommand"));

        // Shooter trim controls
        copilot.trimShooterUp.onTrue(
                Commands.runOnce(() -> RobotState.getInstance().incrementShooterRPMOffset(10))
                        .ignoringDisable(true));
        copilot.trimShooterDown.onTrue(
                Commands.runOnce(() -> RobotState.getInstance().incrementShooterRPMOffset(-10))
                        .ignoringDisable(true));

        // Shooter tuning bindings
        if (Constants.TUNING_MODE) {
            copilot.tuneShooter.toggleOnTrue(
                    ShootingCommands.tuneShooterCommand(turret, shooter, hood).withName("TuneShooterCommand"));
            copilot.tuningFeed.whileTrue(
                    FeedingCommands.feedCommand(turret, hood, spindexer).alongWith(intake.jiggleCommand()));
        }

        // LED bindings
        Supplier<LEDPattern> disabledPattern = () -> {
            if (!DriverStation.isDSAttached())
                return LEDPattern.solid(Color.kWhite).breathe(Seconds.of(1.5));
            else if (!turret.isZeroed()) return CustomLEDPatterns.strobe(Color.kRed);
            return CustomLEDPatterns.transFlag().scrollAtRelativeSpeed(Hertz.of(0.5));
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
        MatchTriggers.endgame()
                .onTrue(leds.viewFull
                        .commandShowPattern(CustomLEDPatterns.strobe(Color.kWhite))
                        .withTimeout(1.0));
    }

    private void configureAutoRoutines() {
        autoSelector.addAuto("Configured Auto", autoConfigurator::getSelectedAuto);
        autoSelector.addAuto("Zero Mechanisms", autoPresets::zeroMechanisms);
        autoSelector.addAuto("Shoot Preload", autoPresets::shootPreload);
        autoSelector.addAuto("Left Bump Depot", autoPresets::leftBumpDepot);

        autoSelector.addAuto("Left Trench 1 Sweep", () -> autoPresets.singleSweep(StartingSide.LEFT, false));
        autoSelector.addAuto("Left Trench 1 Sweep Sprint", () -> autoPresets.singleSweep(StartingSide.LEFT, true));
        autoSelector.addAuto("Right Trench 1 Sweep", () -> autoPresets.singleSweep(StartingSide.RIGHT, false));
        autoSelector.addAuto("Right Trench 1 Sweep Sprint", () -> autoPresets.singleSweep(StartingSide.RIGHT, true));

        autoSelector.addAuto("Left Trench 2 Sweep", () -> autoPresets.doubleSweep(StartingSide.LEFT, false, false));
        autoSelector.addAuto(
                "Left Trench 2 Sweep Sprint", () -> autoPresets.doubleSweep(StartingSide.LEFT, false, true));
        autoSelector.addAuto("Left Trench 2 Sweep Hook", () -> autoPresets.doubleSweep(StartingSide.LEFT, true, false));
        autoSelector.addAuto(
                "Left Trench 2 Sweep Hook Sprint", () -> autoPresets.doubleSweep(StartingSide.LEFT, true, true));
        autoSelector.addAuto("Right Trench 2 Sweep", () -> autoPresets.doubleSweep(StartingSide.RIGHT, false, false));
        autoSelector.addAuto(
                "Right Trench 2 Sweep Sprint", () -> autoPresets.doubleSweep(StartingSide.RIGHT, false, true));
        autoSelector.addAuto(
                "Right Trench 2 Sweep Hook", () -> autoPresets.doubleSweep(StartingSide.RIGHT, true, false));
        autoSelector.addAuto(
                "Right Trench 2 Sweep Hook Sprint", () -> autoPresets.doubleSweep(StartingSide.RIGHT, true, true));

        // Characterization routines
        if (Constants.TUNING_MODE) {
            autoSelector.addAuto("Test", autoPresets::testAuto);
            autoSelector.addCmd(
                    "Drive FF Characterization",
                    () -> CharacterizationCommands.feedforward(
                            drivetrain::runCharacterization, drivetrain::getFFCharacterizationVelocity, drivetrain));
            autoSelector.addCmd(
                    "Drive Wheel Radius Characterization",
                    () -> CharacterizationCommands.wheelRadius(
                            input -> drivetrain.runVelocity(new ChassisSpeeds(0.0, 0.0, input)),
                            () -> RobotState.getInstance().getRobotHeading().getRadians(),
                            drivetrain::getWheelRadiusCharacterizationPositions,
                            drivetrain));
            autoSelector.addCmd(
                    "Intake FF Characterization",
                    () -> CharacterizationCommands.feedforward(
                            intake::setPivotVoltage, intake::getPivotVelocityRPS, intake));
            autoSelector.addCmd(
                    "Shooter FF Characterization",
                    () -> CharacterizationCommands.feedforward(
                            shooter::setVoltage, () -> shooter.getVelocityRPM() / 60, shooter));
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
        addPeriodicCallback(robotState::periodic, "RobotStatePeriodic");
        addPeriodicCallback(HubShiftUtil::periodic, "HubShiftPeriodic");
        addPeriodicCallback(MechanismVisualizer::periodic, "MechanismVisualizerPeriodic");
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            addPeriodicCallback(SimState.getInstance()::update, "SimulationUpdate");
        }
    }

    private void addPeriodicCallback(Runnable callback, String name) {
        CommandScheduler.getInstance()
                .schedule(Commands.run(() -> {
                            LoggedTracer.reset();
                            callback.run();
                            LoggedTracer.record(name);
                        })
                        .withName(name)
                        .ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoSelector.getSelectedAuto().command();
    }
}
