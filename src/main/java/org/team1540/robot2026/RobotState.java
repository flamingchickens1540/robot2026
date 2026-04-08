package org.team1540.robot2026;

import static org.team1540.robot2026.subsystems.turret.TurretConstants.*;
import static org.team1540.robot2026.subsystems.vision.AprilTagVisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleFunction;
import java.util.function.DoubleUnaryOperator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.autos.AutoRoutineData;
import org.team1540.robot2026.subsystems.drive.DrivetrainConstants;
import org.team1540.robot2026.subsystems.hood.HoodConstants;
import org.team1540.robot2026.subsystems.vision.AprilTagVisionIO;
import org.team1540.robot2026.util.AimingParameters;
import org.team1540.robot2026.util.AllianceFlipUtil;
import org.team1540.robot2026.util.LoggedTunableNumber;

public class RobotState {
    private static RobotState instance = null;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    private static final LoggedTunableNumber aimingPhaseDelay = new LoggedTunableNumber("Aiming/PhaseDelay", 0.03);
    private static final LoggedTunableNumber shuffleTargetX =
            new LoggedTunableNumber("Aiming/ShuffleX", FieldConstants.LinesVertical.starting - 1.0);
    private static final LoggedTunableNumber trenchAvoidanceRetractionTime =
            new LoggedTunableNumber("TrenchAvoidance/RetractionTime", 0.3);

    private Rotation2d lastGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };
    private ChassisSpeeds lastVelocity = new ChassisSpeeds();

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(DrivetrainConstants.getModuleTranslations());
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, lastGyroRotation, lastModulePositions, Pose2d.kZero);
    private final Timer poseResetTimer = new Timer();

    private Rotation2d lastTurretAngle = Rotation2d.kZero;
    private double lastTurretVelocityRadPerSec = 0.0;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(0.5);
    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer = TimeInterpolatableBuffer.createBuffer(0.5);

    private final Field2d field = new Field2d();
    private Pose2d[] activeTrajectory;

    private final InterpolatingTreeMap<Double, Rotation2d> hubHoodAngleMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private final InterpolatingDoubleTreeMap hubShooterSpeedMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hubTOFMap = new InterpolatingDoubleTreeMap();

    private final InterpolatingTreeMap<Double, Rotation2d> shuffleHoodAngleMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private final InterpolatingDoubleTreeMap shuffleShooterSpeedMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shuffleTOFMap = new InterpolatingDoubleTreeMap();

    @AutoLogOutput(key = "Aiming/ShooterRPMOffset")
    private double shooterRPMOffset = 20.0;

    @AutoLogOutput(key = "Aiming/Hub/LastParameters")
    private AimingParameters lastHubAimingParameters;

    @AutoLogOutput(key = "Aiming/Shuffle/LastParameters")
    private AimingParameters lastShuffleAimingParameters;

    private AutoRoutineData selectedAuto;

    private final Alert autoStartPositionAlert =
            new Alert("Robot is not near the selected auto's starting position", Alert.AlertType.kWarning);
    private final Alert autoStartRotationAlert =
            new Alert("Robot is not facing the correct direction for the selected auto", Alert.AlertType.kWarning);

    private RobotState() {
        poseResetTimer.start();
        SmartDashboard.putData(field);

        // Set up interpolations
        hubHoodAngleMap.put(3.169, Rotation2d.fromDegrees(23.4));
        hubHoodAngleMap.put(2.543, Rotation2d.fromDegrees(21.1));
        hubHoodAngleMap.put(1.421, Rotation2d.fromDegrees(15));
        hubHoodAngleMap.put(5.540, Rotation2d.fromDegrees(29));
        hubHoodAngleMap.put(4.155, Rotation2d.fromDegrees(26.5));
        hubHoodAngleMap.put(6.455, Rotation2d.fromDegrees(29.0));

        hubShooterSpeedMap.put(3.175, 2154.0);
        hubShooterSpeedMap.put(2.540, 2046.0);
        hubShooterSpeedMap.put(1.439, 1778.0);
        hubShooterSpeedMap.put(5.550, 2670.0);
        hubShooterSpeedMap.put(4.155, 2300.0);
        hubShooterSpeedMap.put(6.350, 3050.0);

        hubTOFMap.put(3.175, 1.099743391);
        hubTOFMap.put(2.540, 1.027414461);
        hubTOFMap.put(1.439, 0.8991000919);
        hubTOFMap.put(5.550, 1.324787032);
        hubTOFMap.put(4.155, 1.188565533);
        hubTOFMap.put(6.35, 1.366890583);

        shuffleHoodAngleMap.put(2.412, Rotation2d.fromDegrees(30));
        shuffleHoodAngleMap.put(4.466, Rotation2d.fromDegrees(35));
        shuffleHoodAngleMap.put(7.590, Rotation2d.fromDegrees(40));
        shuffleHoodAngleMap.put(12.132, Rotation2d.fromDegrees(45));

        shuffleShooterSpeedMap.put(2.412, 1323.0);
        shuffleShooterSpeedMap.put(4.466, 1923.0);
        shuffleShooterSpeedMap.put(7.590, 2471.0);
        shuffleShooterSpeedMap.put(12.132, 4414.0);

        shuffleTOFMap.put(2.412, 0.8041877681);
        shuffleTOFMap.put(4.466, 1.04664185);
        shuffleTOFMap.put(7.590, 1.280397483);
        shuffleTOFMap.put(12.132, 1.50628167);
        AutoLogOutputManager.addObject(this);
    }

    public void periodic() {
        SmartDashboard.putString("Aiming/Shooter RPM Offset", String.format("%.1f", shooterRPMOffset));
        clearAimingParameters();

        if (DriverStation.isAutonomous() && DriverStation.isDisabled()) {
            autoStartPositionAlert.set(selectedAuto != null
                    && selectedAuto.startingPose().isPresent()
                    && getEstimatedPose()
                                    .getTranslation()
                                    .getDistance(AllianceFlipUtil.apply(
                                                    selectedAuto.startingPose().get())
                                            .getTranslation())
                            > 0.5);
            autoStartRotationAlert.set(selectedAuto != null
                    && selectedAuto.startingPose().isPresent()
                    && !autoStartRotationAlert.get()
                    && Math.abs(getEstimatedPose()
                                    .getRotation()
                                    .minus(AllianceFlipUtil.apply(
                                                    selectedAuto.startingPose().get())
                                            .getRotation())
                                    .getDegrees())
                            > 10.0);
            field.getObject("trajectory").setPoses(AllianceFlipUtil.apply(selectedAuto.trajectoryPoints()));
        } else {
            autoStartPositionAlert.set(false);
            autoStartRotationAlert.set(false);
            field.getObject("trajectory").setPoses();
        }
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void addOdometryObservation(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle, double timestamp) {
        lastModulePositions = modulePositions;
        lastGyroRotation = gyroAngle;

        poseEstimator.updateWithTime(timestamp, gyroAngle, modulePositions);
        poseBuffer.addSample(timestamp, poseEstimator.getEstimatedPosition());
        field.setRobotPose(getEstimatedPose());
    }

    public void resetPose(Pose2d newPose) {
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) SimState.getInstance().resetSimPose(newPose);
        poseEstimator.resetPosition(lastGyroRotation, lastModulePositions, newPose);
        poseBuffer.clear();
        poseResetTimer.restart();
    }

    @AutoLogOutput(key = "Odometry/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Odometry/TurretPose")
    public Pose2d getTurretPose() {
        return getEstimatedPose().transformBy(new Transform2d(ROBOT_TO_TURRET_2D.getTranslation(), lastTurretAngle));
    }

    public Rotation2d getRobotHeading() {
        return getEstimatedPose().getRotation();
    }

    public Rotation2d getTurretAngle() {
        return lastTurretAngle;
    }

    public Rotation2d getFieldRelativeTurretAngle() {
        return getTurretAngle().plus(getRobotHeading());
    }

    public double getTurretVelocityRadPerSec() {
        return lastTurretVelocityRadPerSec;
    }

    public void addVelocityObservation(ChassisSpeeds velocity) {
        lastVelocity = velocity;
    }

    @AutoLogOutput(key = "Odometry/RobotVelocity")
    public ChassisSpeeds getRobotVelocity() {
        return lastVelocity;
    }

    @AutoLogOutput(key = "Odometry/FieldRelativeVelocity")
    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(lastVelocity, getRobotHeading());
    }

    @AutoLogOutput(key = "Odometry/RobotSpeedMPS")
    public double getRobotSpeedMPS() {
        return Math.hypot(lastVelocity.vxMetersPerSecond, lastVelocity.vyMetersPerSecond);
    }

    public void setActiveTrajectory(Pose2d... trajectory) {
        activeTrajectory = trajectory;
        Logger.recordOutput("Odometry/Trajectory/Current", trajectory);
    }

    public void clearActiveTrajectory() {
        activeTrajectory = null;
        Logger.recordOutput("Odometry/Trajectory/Current", new Pose2d[0]);
    }

    public void setSelectedAuto(AutoRoutineData auto) {
        selectedAuto = auto;
    }

    public void setTrajectoryTarget(Pose2d target) {
        Logger.recordOutput("Odometry/Trajectory/TargetPose", target);
    }

    public void addTurretObservation(Rotation2d turretAngle, double turretVelocityRadPerSec, double timestamp) {
        lastTurretAngle = turretAngle;
        lastTurretVelocityRadPerSec = turretVelocityRadPerSec;
        turretAngleBuffer.addSample(timestamp, turretAngle);
    }

    public boolean addVisionMeasurement(AprilTagVisionIO.PoseObservation visionPose) {
        if (shouldAcceptVision(visionPose) && poseResetTimer.hasElapsed(0.1)) {
            poseEstimator.addVisionMeasurement(
                    visionPose.estimatedPoseMeters().toPose2d(),
                    visionPose.timestampSecs(),
                    getStdDevs(visionPose, XY_STD_DEV_COEFF, ROT_STD_DEV_COEFF));
            return true;
        }
        return false;
    }

    public boolean addTurretVisionMeasurement(AprilTagVisionIO.PoseObservation turretPose) {
        if (shouldAcceptVision(turretPose) && poseResetTimer.hasElapsed(0.1)) {
            Rotation2d turretAngleAtMeasurement =
                    turretAngleBuffer.getSample(turretPose.timestampSecs()).orElse(lastTurretAngle);
            Pose2d robotPose = turretPose
                    .estimatedPoseMeters()
                    .transformBy(new Transform3d(
                                    ROBOT_TO_TURRET_3D.getTranslation(),
                                    new Rotation3d(0.0, 0.0, turretAngleAtMeasurement.getRadians()))
                            .inverse())
                    .toPose2d();
            poseEstimator.addVisionMeasurement(
                    robotPose,
                    turretPose.timestampSecs(),
                    getStdDevs(turretPose, TURRET_XY_STD_DEV_COEFF, TURRET_ROT_STD_DEV_COEFF));
            return true;
        }
        return false;
    }

    private boolean shouldAcceptVision(AprilTagVisionIO.PoseObservation poseObservation) {
        Pose3d estimatedPose = poseObservation.estimatedPoseMeters();
        return poseObservation.numTagsSeen() >= MIN_ACCEPTED_NUM_TAGS // Must see sufficient tags
                && (poseObservation.numTagsSeen() > 1
                        || poseObservation.ambiguity() < MAX_AMBIGUITY) // Must be multiple tags or low ambiguity
                // Must be within field roughly
                && estimatedPose.getX() >= -MAX_OUTSIDE_OF_FIELD_TOLERANCE
                && estimatedPose.getX() <= FieldConstants.fieldLength + MAX_OUTSIDE_OF_FIELD_TOLERANCE
                && estimatedPose.getY() >= -MAX_OUTSIDE_OF_FIELD_TOLERANCE
                && estimatedPose.getY() <= FieldConstants.fieldWidth + MAX_OUTSIDE_OF_FIELD_TOLERANCE
                // Must not be actively flying
                && Math.abs(estimatedPose.getZ()) <= MAX_ROBOT_Z_TOLERANCE;
    }

    private Matrix<N3, N1> getStdDevs(
            AprilTagVisionIO.PoseObservation poseObservation, double xyCoeff, double rotCoeff) {
        double xyStdDev = xyCoeff * Math.pow(poseObservation.avgTagDistance(), 2.0) / poseObservation.numTagsSeen();
        double rotStdDev = rotCoeff * Math.pow(poseObservation.avgTagDistance(), 2.0) / poseObservation.numTagsSeen();
        return VecBuilder.fill(
                xyStdDev,
                xyStdDev,
                DriverStation.isEnabled() && poseObservation.numTagsSeen() <= 1 ? Double.POSITIVE_INFINITY : rotStdDev);
    }

    public enum TargetingMode {
        HUB,
        SHUFFLE
    }

    public TargetingMode getTargetingMode() {
        return AllianceFlipUtil.apply(getEstimatedPose()).getX()
                        < FieldConstants.LinesVertical.allianceZone + Constants.BUMPER_LENGTH_X_METERS
                ? TargetingMode.HUB
                : TargetingMode.SHUFFLE;
    }

    private Translation2d getShuffleTarget() {
        if (AllianceFlipUtil.apply(getEstimatedPose()).getY() < FieldConstants.LinesHorizontal.center) {
            return AllianceFlipUtil.apply(new Translation2d(
                    shuffleTargetX.get(),
                    MathUtil.clamp(
                            AllianceFlipUtil.apply(getEstimatedPose()).getY(),
                            FieldConstants.LinesHorizontal.rightTrenchOpenStart,
                            FieldConstants.LinesHorizontal.rightBumpMiddle)));
        } else {
            return AllianceFlipUtil.apply(new Translation2d(
                    shuffleTargetX.get(),
                    MathUtil.clamp(
                            AllianceFlipUtil.apply(getEstimatedPose()).getY(),
                            FieldConstants.LinesHorizontal.leftBumpMiddle,
                            FieldConstants.LinesHorizontal.leftTrenchOpenEnd)));
        }
    }

    private AimingParameters getCompensatedAimingParameters(
            Translation2d target,
            DoubleFunction<Rotation2d> hoodAngleMap,
            DoubleUnaryOperator shooterSpeedMap,
            DoubleUnaryOperator tofMap,
            String loggingKey) {
        double phaseDelay = aimingPhaseDelay.getAsDouble();
        Pose2d estimatedPose = getEstimatedPose();
        ChassisSpeeds velocity = getRobotVelocity();
        estimatedPose = estimatedPose.exp(new Twist2d(
                velocity.vxMetersPerSecond * phaseDelay,
                velocity.vyMetersPerSecond * phaseDelay,
                velocity.omegaRadiansPerSecond * phaseDelay));

        Pose2d turretPose = estimatedPose.transformBy(ROBOT_TO_TURRET_2D);
        double targetDistance = target.getDistance(turretPose.getTranslation());

        Logger.recordOutput("Aiming/" + loggingKey + "/ActualTarget", target);
        Logger.recordOutput("Aiming/" + loggingKey + "/ActualTargetDistanceMeters", targetDistance);

        Translation2d turretVelocity = new Translation2d(
                        getFieldRelativeVelocity().vxMetersPerSecond, getFieldRelativeVelocity().vyMetersPerSecond)
                .plus(new Translation2d(
                        velocity.omegaRadiansPerSecond
                                * ROBOT_TO_TURRET_2D.getTranslation().getNorm(),
                        estimatedPose.getRotation().rotateBy(Rotation2d.kCW_90deg)));

        double timeOfFlight = tofMap.applyAsDouble(targetDistance);
        Pose2d lookaheadPose = turretPose;
        double lookaheadDistance = targetDistance;

        for (int i = 0; i < 20; i++) {
            double offsetX = turretVelocity.getX() * timeOfFlight;
            double offsetY = turretVelocity.getY() * timeOfFlight;
            lookaheadPose = new Pose2d(
                    turretPose.getTranslation().plus(new Translation2d(offsetX, offsetY)), turretPose.getRotation());
            lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
            timeOfFlight = tofMap.applyAsDouble(lookaheadDistance);
        }

        double turretVelocityFFRadPerSec = -target.minus(lookaheadPose.getTranslation())
                                .rotateBy(Rotation2d.kCCW_90deg)
                                .dot(turretVelocity)
                        / (lookaheadDistance * lookaheadDistance)
                - getRobotVelocity().omegaRadiansPerSecond;

        Logger.recordOutput(
                "Aiming/" + loggingKey + "/CompensatedTurretPose",
                lookaheadPose.rotateAround(lookaheadPose.getTranslation(), getFieldRelativeTurretAngle()));
        Logger.recordOutput(
                "Aiming/" + loggingKey + "/CompensatedRobotPose",
                lookaheadPose.transformBy(ROBOT_TO_TURRET_2D.inverse()));
        Logger.recordOutput(
                "Aiming/" + loggingKey + "/CompensatedTarget",
                target.plus(turretPose.getTranslation().minus(lookaheadPose.getTranslation())));
        Logger.recordOutput("Aiming/" + loggingKey + "/CompensatedTargetDistanceMeters", lookaheadDistance);
        Logger.recordOutput(
                "Aiming/" + loggingKey + "/TurretVelocityFFRPS", Units.radiansToRotations(turretVelocityFFRadPerSec));

        return new AimingParameters(
                target.minus(lookaheadPose.getTranslation()).getAngle(),
                turretVelocityFFRadPerSec,
                hoodAngleMap.apply(lookaheadDistance),
                shooterSpeedMap.applyAsDouble(lookaheadDistance) + shooterRPMOffset);
    }

    public AimingParameters getHubAimingParameters() {
        if (lastHubAimingParameters != null) return lastHubAimingParameters;
        lastHubAimingParameters = getCompensatedAimingParameters(
                AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()),
                distance -> shouldLowerHood() ? HoodConstants.MIN_ANGLE : hubHoodAngleMap.get(distance),
                hubShooterSpeedMap::get,
                hubTOFMap::get,
                "Hub");
        return lastHubAimingParameters;
    }

    public AimingParameters getShuffleAimingParameters() {
        if (lastShuffleAimingParameters != null) return lastShuffleAimingParameters;
        Translation2d shuffleTarget = getShuffleTarget();
        lastShuffleAimingParameters = getCompensatedAimingParameters(
                shuffleTarget,
                distance -> shouldLowerHood() ? HoodConstants.MIN_ANGLE : shuffleHoodAngleMap.get(distance),
                shuffleShooterSpeedMap::get,
                shuffleTOFMap::get,
                "Shuffle");
        return lastShuffleAimingParameters;
    }

    public AimingParameters getAimingParameters() {
        return switch (getTargetingMode()) {
            case HUB -> getHubAimingParameters();
            case SHUFFLE -> getShuffleAimingParameters();
        };
    }

    private void clearAimingParameters() {
        lastHubAimingParameters = null;
        lastShuffleAimingParameters = null;
    }

    public void incrementShooterRPMOffset(double rpm) {
        shooterRPMOffset += rpm;
    }

    @AutoLogOutput(key = "TrenchAvoidance/Active")
    public boolean shouldLowerHood() {
        Pose2d robotPose = getEstimatedPose();
        if (robotPose.getY() >= FieldConstants.LinesHorizontal.rightTrenchOpenStart
                && robotPose.getY() < FieldConstants.LinesHorizontal.leftTrenchOpenEnd)
            return false; // Don't lower hood if not in trench strips

        Pose2d turretPose = getTurretPose();
        ChassisSpeeds robotVelocity = getFieldRelativeVelocity();

        Rectangle2d[] trenches = FieldConstants.Regions.trenches;
        for (Rectangle2d trench : trenches) {
            if (trench.contains(turretPose.getTranslation())) return true; // Lower hood if directly under trench

            Translation2d closestPoint = trench.nearest(turretPose.getTranslation());
            Translation2d turretToTrench = closestPoint.minus(turretPose.getTranslation());

            Translation2d velocityVector =
                    new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
            double closingSpeed = velocityVector.dot(turretToTrench.div(
                    turretToTrench.getNorm())); // Direction of robot velocity moving towards the trench
            Logger.recordOutput("TrenchAvoidance/ClosingSpeed", closingSpeed);
            if (closingSpeed <= 0.0) continue; // Moving away from trench, no risk of entering

            double regionWidth = trench.getXWidth() + 2 * trenchAvoidanceRetractionTime.get() * closingSpeed;
            Rectangle2d avoidanceRegion = new Rectangle2d(trench.getCenter(), regionWidth, trench.getYWidth());
            if (avoidanceRegion.contains(turretPose.getTranslation()))
                return true; // Lower hood if in avoidance region around trench
        }
        return false;
    }
}
