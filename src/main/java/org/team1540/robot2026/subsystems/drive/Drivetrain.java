package org.team1540.robot2026.subsystems.drive;

import static org.team1540.robot2026.subsystems.drive.DrivetrainConstants.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.RobotState;
import org.team1540.robot2026.SimState;
import org.team1540.robot2026.generated.TunerConstants;
import org.team1540.robot2026.subsystems.PieceDetection.PieceDetection;
import org.team1540.robot2026.util.AllianceFlipUtil;
import org.team1540.robot2026.util.JoystickUtil;
import org.team1540.robot2026.util.LoggedTracer;
import org.team1540.robot2026.util.LoggedTunableNumber;
import org.team1540.robot2026.util.swerve.TrajectoryController;

public class Drivetrain extends SubsystemBase {
    private static boolean hasInstance = false;
    static final Lock odometryLock = new ReentrantLock();

    private static final LoggedTunableNumber translationKP = new LoggedTunableNumber("Drivetrain/Translation/kP", 6.0);
    private static final LoggedTunableNumber translationKI = new LoggedTunableNumber("Drivetrain/Translation/kI", 0.0);
    private static final LoggedTunableNumber translationKD = new LoggedTunableNumber("Drivetrain/Translation/kD", 0.0);

    private static final LoggedTunableNumber rotationKP = new LoggedTunableNumber("Drivetrain/Rotation/kP", 3.3);
    private static final LoggedTunableNumber rotationKI = new LoggedTunableNumber("Drivetrain/Rotation/kI", 0.0);
    private static final LoggedTunableNumber rotationKD = new LoggedTunableNumber("Drivetrain/Rotation/kD", 0.3);

    private static final LoggedTunableNumber sampleRejectionThreshold =
            new LoggedTunableNumber("Drivetrain/Odometry/SampleRejectionThreshold", 2.0);

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    private Rotation2d fieldOrientationOffset = Rotation2d.kZero;
    private Rotation2d rawGyroRotation = Rotation2d.kZero;

    private final SwerveDriveKinematics kinematics = RobotState.getInstance().getKinematics();
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4];
    private double lastOdometryUpdateTimeSecs = 0.0;

    private final PieceDetection pieceDetection = new PieceDetection();

    private final ProfiledPIDController headingController = new ProfiledPIDController(
            rotationKP.get(),
            rotationKI.get(),
            rotationKD.get(),
            new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RAD_PER_SEC, 10));
    private final TrajectoryController trajectoryController = new TrajectoryController(
            translationKP.get(),
            translationKI.get(),
            translationKD.get(),
            rotationKP.get(),
            rotationKI.get(),
            rotationKD.get());

    private final Alert gyroDisconnectedAlert = new Alert("Gyro disconnected", Alert.AlertType.kError);

    public Drivetrain(
            GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        if (hasInstance) throw new IllegalStateException("Instance of drivetrain already exists");
        hasInstance = true;

        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, Module.MountPosition.FL, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, Module.MountPosition.FR, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, Module.MountPosition.BL, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, Module.MountPosition.BR, TunerConstants.BackRight);

        for (int i = 0; i < 4; i++) {
            lastModulePositions[i] = modules[i].getPosition();
        }

        headingController.setTolerance(Math.toRadians(1.0));
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        OdometryThread.getInstance().start();
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        for (Module module : modules) module.periodic();
        odometryLock.unlock();

        Logger.processInputs("Drivetrain/Gyro", gyroInputs);

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        int rejectedSamples = 0;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
            }

            // Filter odometry data based on wheel deltas
            boolean acceptMeasurement = true;
            double dt = sampleTimestamps[i] - lastOdometryUpdateTimeSecs;
            for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                double velocity = moduleDeltas[moduleIndex].distanceMeters / dt;
                double turnVelocity = modulePositions[moduleIndex]
                                .angle
                                .minus(lastModulePositions[moduleIndex].angle)
                                .getRadians()
                        / dt;
                if (Math.abs(velocity) > MAX_LINEAR_SPEED_MPS * sampleRejectionThreshold.get()
                        || Math.abs(turnVelocity) > MAX_STEER_SPEED_RAD_PER_SEC * sampleRejectionThreshold.get()) {
                    acceptMeasurement = false;
                    break;
                }
            }
            // Accept measurements if delta is not too large
            if (acceptMeasurement) {
                if (gyroInputs.connected) rawGyroRotation = gyroInputs.odometryYawPositions[i];
                else {
                    // If gyro is disconnected, use kinematics to estimate rotation
                    Twist2d twist = kinematics.toTwist2d(lastModulePositions, modulePositions);
                    rawGyroRotation = rawGyroRotation.plus(Rotation2d.fromRadians(twist.dtheta));
                }
                RobotState.getInstance().addOdometryObservation(modulePositions, rawGyroRotation, sampleTimestamps[i]);
                lastModulePositions = modulePositions;
                lastOdometryUpdateTimeSecs = sampleTimestamps[i];
            } else {
                rejectedSamples++;
            }
        }
        Logger.recordOutput("Odometry/RejectedSamples", rejectedSamples);

        // Update robot velocities
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
        speeds.omegaRadiansPerSecond =
                gyroInputs.connected ? gyroInputs.yawVelocityRadPerSec : speeds.omegaRadiansPerSecond;
        RobotState.getInstance().setRobotVelocity(speeds);

        // Stop modules when disabled
        if (DriverStation.isDisabled()) {
            for (Module module : modules) {
                module.stop();
            }
            Logger.recordOutput(
                    "Drivetrain/SwerveStates/Setpoints",
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState());
        }

        // Update tunable PID constants
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> trajectoryController.setTranslationPID(
                        translationKP.get(), translationKI.get(), translationKD.get()),
                translationKP,
                translationKI,
                translationKD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    trajectoryController.setHeadingPID(rotationKP.get(), rotationKI.get(), rotationKD.get());
                    headingController.setPID(rotationKP.get(), rotationKI.get(), rotationKD.get());
                },
                rotationKP,
                rotationKI,
                rotationKD);

        // Update alerts
        gyroDisconnectedAlert.set(!gyroInputs.connected);

        // Log active command
        Command activeCmd = CommandScheduler.getInstance().requiring(this);
        Logger.recordOutput(
                "Drivetrain/ActiveCommand",
                activeCmd != null ? activeCmd.getName() + "_" + Integer.toHexString(activeCmd.hashCode()) : "None");

        LoggedTracer.record("Drivetrain");
    }

    /** Runs the drivetrain at the given velocity */
    public void runVelocity(ChassisSpeeds velocity) {
        SwerveModuleState[] setpoints =
                kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(velocity, Constants.LOOP_PERIOD_SECS));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                setpoints, MAX_LINEAR_SPEED_MPS); // Ensure wheel speeds are physically reachable
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpoints[i]);
        }
        Logger.recordOutput("Drivetrain/SwerveStates/Setpoints", setpoints);
    }

    /** Runs the drivetrain such that it follows the given trajectory sample */
    public void followTrajectory(SwerveSample trajectorySample) {
        RobotState.getInstance().setTrajectoryTarget(trajectorySample.getPose());
        runVelocity(trajectoryController.calculate(RobotState.getInstance().getEstimatedPose(), trajectorySample));
    }

    /** Align all modules forward and runs the drive at the given open-loop input for FF characterization*/
    public void runCharacterization(double input) {
        for (Module module : modules) {
            module.runCharacterization(input);
        }
    }

    public double getFFCharacterizationVelocity() {
        double totalVelocity = 0.0;
        for (Module module : modules) totalVelocity += module.getFFCharacterizationVelocity();
        return totalVelocity / modules.length;
    }

    public double[] getWheelRadiusCharacterizationPositions() {
        return Arrays.stream(modules)
                .mapToDouble(Module::getWheelRadiusCharacterizationPosition)
                .toArray();
    }

    /** Stops the drivetrain */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /** Stops the drivetrain with wheels pointed in an X pattern */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        Translation2d[] modulePositions = getModuleTranslations();
        for (int i = 0; i < 4; i++) headings[i] = modulePositions[i].getAngle();
        kinematics.resetHeadings(headings);
        stop();
    }

    @AutoLogOutput(key = "Drivetrain/SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Zeros the drivetrain's field orientation based on the robot's estimated heading */
    public void zeroFieldOrientation() {
        fieldOrientationOffset = rawGyroRotation.minus(
                AllianceFlipUtil.apply(RobotState.getInstance().getRobotHeading()));
    }

    /** Manually zero the drivetrain's field orientation to where the robot is pointing*/
    public void zeroFieldOrientationManual() {
        fieldOrientationOffset = rawGyroRotation;
    }

    public void setBrakeMode(boolean enabled) {
        for (Module module : modules) module.setBrakeMode(enabled);
    }

    public Command percentDriveCommand(Supplier<Translation2d> translationPercent, DoubleSupplier omegaPercent) {
        return percentDriveCommand(translationPercent, omegaPercent, () -> true);
    }

    public Command percentDriveCommand(
            Supplier<Translation2d> translationPercent, DoubleSupplier omegaPercent, BooleanSupplier fieldRelative) {
        return run(() -> {
                    ChassisSpeeds velocity = new ChassisSpeeds(
                            translationPercent.get().getX() * MAX_LINEAR_SPEED_MPS,
                            translationPercent.get().getY() * MAX_LINEAR_SPEED_MPS,
                            omegaPercent.getAsDouble() * MAX_ANGULAR_SPEED_RAD_PER_SEC);
                    if (fieldRelative.getAsBoolean()) {
                        velocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                                velocity, rawGyroRotation.minus(fieldOrientationOffset));
                    }
                    runVelocity(velocity);
                })
                .finallyDo(this::stop)
                .withName("PercentDriveCommand");
    }

    public Command teleopDriveCommand(XboxController controller) {
        return percentDriveCommand(
                        () -> JoystickUtil.deadzonedJoystickTranslation(
                                -controller.getLeftY(), -controller.getLeftX(), 0.1),
                        () -> JoystickUtil.smartDeadzone(-controller.getRightX(), 0.1))
                .withName("TeleopDriveCommand");
    }
    public Command teleopDriveCommandWithPieceDetection(XboxController controller, double kp, double ki, double kd){
        return percentDriveCommand(
                () -> JoystickUtil.deadzonedJoystickTranslation(
                        -controller.getLeftY(), -controller.getLeftX(), 0.1),
                () -> JoystickUtil.smartDeadzone(-controller.getRightX()+pieceDetection.getPieceDetectionAngle(kp, ki, kd), 0.1))
                .withName("teleopDriveCommandWithPieceDetection");
    }

    public static Drivetrain createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL)
            DriverStation.reportWarning("Using real drivetrain on simulated robot", false);
        return new Drivetrain(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
    }

    public static Drivetrain createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL)
            DriverStation.reportWarning("Using simulated drivetrain on real robot", false);

        SwerveDriveSimulation driveSim = SimState.getInstance().getDriveSim();
        return new Drivetrain(
                new GyroIOSim(driveSim.getGyroSimulation()),
                new ModuleIOSim(TunerConstants.FrontLeft, driveSim.getModules()[0]),
                new ModuleIOSim(TunerConstants.FrontRight, driveSim.getModules()[1]),
                new ModuleIOSim(TunerConstants.BackLeft, driveSim.getModules()[2]),
                new ModuleIOSim(TunerConstants.BackRight, driveSim.getModules()[3]));
    }

    public static Drivetrain createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy drivetrain on real robot", false);
        return new Drivetrain(
                new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
    }
}
