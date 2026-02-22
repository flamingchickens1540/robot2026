package org.team1540.robot2026;

import static org.team1540.robot2026.subsystems.vision.AprilTagVisionConstants.*;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.subsystems.drive.DrivetrainConstants;
import org.team1540.robot2026.subsystems.turret.TurretConstants;
import org.team1540.robot2026.subsystems.vision.AprilTagVisionIO;
import org.team1540.robot2026.util.AimingParameters;
import org.team1540.robot2026.util.AllianceFlipUtil;

public class RobotState {
    private static RobotState instance = null;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

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

    private final Field2d field = new Field2d();
    private Pose2d[] activeTrajectory;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(0.5);
    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer = TimeInterpolatableBuffer.createBuffer(0.5);

    private final InterpolatingTreeMap<Double, Rotation2d> hubHoodAngleMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private final InterpolatingDoubleTreeMap hubShooterSpeedMap = new InterpolatingDoubleTreeMap();

    private final InterpolatingTreeMap<Double, Rotation2d> highShuffleHoodAngleMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private final InterpolatingDoubleTreeMap highShuffleShooterSpeedMap = new InterpolatingDoubleTreeMap();

    private final InterpolatingTreeMap<Double, Rotation2d> lowShuffleHoodAngleMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private final InterpolatingDoubleTreeMap lowShuffleShooterSpeedMap = new InterpolatingDoubleTreeMap();

    private final InterpolatingDoubleTreeMap shootOnTheMoveTimeMap = new InterpolatingDoubleTreeMap();

    private RobotState() {
        poseResetTimer.start();
        SmartDashboard.putData(field);

        // Set up interpolations
        hubHoodAngleMap.put(2.713, Rotation2d.fromDegrees(22));
        hubHoodAngleMap.put(1.724, Rotation2d.fromDegrees(17));
        hubHoodAngleMap.put(1.414, Rotation2d.fromDegrees(15));
        hubHoodAngleMap.put(4.888, Rotation2d.fromDegrees(29));
        hubHoodAngleMap.put(3.700, Rotation2d.fromDegrees(25));

        hubShooterSpeedMap.put(2.713, 1400.0);
        hubShooterSpeedMap.put(1.724, 1250.0);
        hubShooterSpeedMap.put(1.414, 1150.0);
        hubShooterSpeedMap.put(4.888, 1650.0);
        hubShooterSpeedMap.put(3.700, 1500.0);

        AutoLogOutputManager.addObject(this);
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

    public Rotation2d getRobotHeading() {
        return getEstimatedPose().getRotation();
    }

    public void setRobotVelocity(ChassisSpeeds velocity) {
        lastVelocity = velocity;
    }

    @AutoLogOutput(key = "Odometry/RobotVelocity")
    public ChassisSpeeds getRobotVelocity() {
        return lastVelocity;
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(lastVelocity, getRobotHeading());
    }

    @AutoLogOutput(key = "Odometry/RobotSpeedMPS")
    public double getRobotSpeedMPS() {
        return Math.hypot(lastVelocity.vxMetersPerSecond, lastVelocity.vyMetersPerSecond);
    }

    public void setActiveTrajectory(Pose2d... trajectory) {
        activeTrajectory = trajectory;
        field.getObject("trajectory").setPoses(trajectory);
        Logger.recordOutput("Odometry/Trajectory/Current", trajectory);
    }

    public void clearActiveTrajectory() {
        activeTrajectory = null;
        field.getObject("trajectory").setPoses();
        Logger.recordOutput("Odometry/Trajectory/Current", new Pose2d[0]);
    }

    public void setTrajectoryTarget(Pose2d target) {
        Logger.recordOutput("Odometry/Trajectory/TargetPose", target);
    }

    public void addTurretObservation(Rotation2d turretAngle, double timestamp) {
        lastTurretAngle = turretAngle;
        turretAngleBuffer.addSample(timestamp, turretAngle);
    }

    public boolean addVisionMeasurement(AprilTagVisionIO.PoseObservation visionPose) {
        if (shouldAcceptVision(visionPose) && poseResetTimer.hasElapsed(0.1)) {
            poseEstimator.addVisionMeasurement(
                    visionPose.estimatedPoseMeters().toPose2d(), visionPose.timestampSecs(), getStdDevs(visionPose));
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
                                    TurretConstants.ROBOT_TO_TURRET_3D.getTranslation(),
                                    new Rotation3d(0.0, 0.0, turretAngleAtMeasurement.getRadians()))
                            .inverse())
                    .toPose2d();
            poseEstimator.addVisionMeasurement(robotPose, turretPose.timestampSecs(), getStdDevs(turretPose));
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

    private Matrix<N3, N1> getStdDevs(AprilTagVisionIO.PoseObservation poseObservation) {
        double xyStdDev =
                XY_STD_DEV_COEFF * Math.pow(poseObservation.avgTagDistance(), 2.0) / poseObservation.numTagsSeen();
        double rotStdDev =
                ROT_STD_DEV_COEFF * Math.pow(poseObservation.avgTagDistance(), 2.0) / poseObservation.numTagsSeen();
        return VecBuilder.fill(
                xyStdDev,
                xyStdDev,
                DriverStation.isEnabled() && poseObservation.numTagsSeen() <= 1 ? Double.POSITIVE_INFINITY : rotStdDev);
    }

    public AimingParameters getHubAimingParameters() {
        Translation2d turretToHub = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                .minus(getEstimatedPose()
                        .transformBy(TurretConstants.ROBOT_TO_TURRET_2D)
                        .getTranslation());
        double distanceToHubMeters = turretToHub.getNorm();
        return new AimingParameters(
                turretToHub.getAngle(),
                -getRobotVelocity().omegaRadiansPerSecond,
                hubHoodAngleMap.get(distanceToHubMeters),
                hubShooterSpeedMap.get(distanceToHubMeters));
    }

    public AimingParameters getHubSOTMAimingParameters() {
        Translation2d turretToHub = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                .minus(getEstimatedPose()
                        .transformBy(TurretConstants.ROBOT_TO_TURRET_2D)
                        .getTranslation());
        double distanceToHubMeters = turretToHub.getNorm();
        double regularShotSpeed = hubShooterSpeedMap.get(distanceToHubMeters);
        Rotation2d regularHoodAngle = Rotation2d.kCCW_90deg.minus(hubHoodAngleMap.get(distanceToHubMeters));

        double thetaH = Math.atan2(
                regularShotSpeed
                                * regularHoodAngle.getCos()
                                * turretToHub.getAngle().getSin()
                        + getRobotVelocity().vyMetersPerSecond,
                regularShotSpeed
                                * regularHoodAngle.getCos()
                                * turretToHub.getAngle().getCos()
                        + turretToHub.getX()
                        + getRobotVelocity().vxMetersPerSecond);
        double thetaV = Math.atan(regularShotSpeed
                * regularHoodAngle.getSin()
                * Math.cos(thetaH)
                / (regularShotSpeed
                                * regularHoodAngle.getCos()
                                * turretToHub.getAngle().getCos()
                        + getRobotVelocity().vxMetersPerSecond));

        return new AimingParameters(
                Rotation2d.fromRadians(thetaH),
                -getRobotVelocity().omegaRadiansPerSecond,
                Rotation2d.kCCW_90deg.minus(Rotation2d.fromRadians(thetaV)),
                hubShooterSpeedMap.get(distanceToHubMeters) * (regularHoodAngle.getSin() / Math.sin(thetaV)));
    }

    public AimingParameters getHighShuffleAimingParameters(Translation2d shuffleTarget) {
        Translation2d turretToTarget = shuffleTarget.minus(getEstimatedPose()
                .transformBy(TurretConstants.ROBOT_TO_TURRET_2D)
                .getTranslation());
        double distanceToTargetMeters = turretToTarget.getNorm();
        return new AimingParameters(
                turretToTarget.getAngle(),
                -getRobotVelocity().omegaRadiansPerSecond,
                Rotation2d.fromDegrees(45),
                0.67 * 5);
    }

    public AimingParameters getLowShuffleAimingParameters(Translation2d shuffleTarget) {
        Translation2d turretToTarget = shuffleTarget.minus(getEstimatedPose()
                .transformBy(TurretConstants.ROBOT_TO_TURRET_2D)
                .getTranslation());
        double distanceToTargetMeters = turretToTarget.getNorm();
        return new AimingParameters(
                turretToTarget.getAngle(),
                -getRobotVelocity().omegaRadiansPerSecond,
                lowShuffleHoodAngleMap.get(distanceToTargetMeters),
                lowShuffleShooterSpeedMap.get(distanceToTargetMeters));
    }

    public AimingParameters getShootOnTheMoveAimingParameters(Translation2d target) {
        Pose2d currentPose = getEstimatedPose();
        Translation2d turretToTarget = target.minus(getEstimatedPose()
                .transformBy(TurretConstants.ROBOT_TO_TURRET_2D)
                .getTranslation());

        double time = shootOnTheMoveTimeMap.get(target.getNorm());

        Translation2d turretMovement = new Translation2d(
                (getFieldRelativeVelocity().vxMetersPerSecond * time),
                (getFieldRelativeVelocity().vyMetersPerSecond * time));
        Translation2d futureTargetPos = turretToTarget.plus(turretMovement);

        return new AimingParameters(
                turretToTarget.getAngle(),
                -getRobotVelocity().omegaRadiansPerSecond,
                lowShuffleHoodAngleMap.get(futureTargetPos.getNorm()),
                lowShuffleShooterSpeedMap.get(futureTargetPos.getNorm()));
    }
}
