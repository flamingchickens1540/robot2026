package org.team1540.robot2026;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
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
import org.team1540.robot2026.subsystems.vision.AprilVisionIO;

import java.util.Optional;

import static org.team1540.robot2026.subsystems.vision.AprilTagVisionConstants.*;

public class RobotState {

    private final Timer resetTimer = new Timer();
    private final TimeInterpolatableBuffer<Pose2d> fusedPoseBuffer = TimeInterpolatableBuffer.createBuffer(2.0);
    private final SingleTagPoseEstimate[] singleTagPoses = new SingleTagPoseEstimate[FieldConstants.aprilTagCount];

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

    private final Field2d field = new Field2d();
    private Pose2d[] activeTrajectory;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(0.5);
    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer = TimeInterpolatableBuffer.createBuffer(0.5);

    private RobotState() {
        poseResetTimer.start();
        SmartDashboard.putData(field);
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
        turretAngleBuffer.addSample(timestamp, turretAngle);
    }

    public boolean addVisionMeasurement(AprilVisionIO.PoseObservation visionPose) {
        if (shouldAcceptVision(visionPose) && resetTimer.hasElapsed(0.1)) {
            poseEstimator.addVisionMeasurement(
                    visionPose.estimatedPoseMeters().toPose2d(), visionPose.timestampSecs(), getStdDevs(visionPose));
            fusedPoseBuffer.addSample(visionPose.timestampSecs(), poseEstimator.getEstimatedPosition());
            return true;
        }
        return false;
    }

    private boolean shouldAcceptVision(AprilVisionIO.PoseObservation poseObservation) {
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

    private Matrix<N3, N1> getStdDevs(AprilVisionIO.PoseObservation poseObservation) {
        double xyStdDev =
                XY_STD_DEV_COEFF * Math.pow(poseObservation.avgTagDistance(), 2.0) / poseObservation.numTagsSeen();
        double rotStdDev =
                ROT_STD_DEV_COEFF * Math.pow(poseObservation.avgTagDistance(), 2.0) / poseObservation.numTagsSeen();
        return VecBuilder.fill(
                xyStdDev,
                xyStdDev,
                DriverStation.isEnabled() && poseObservation.numTagsSeen() <= 1 ? Double.POSITIVE_INFINITY : rotStdDev);
    }

    public void addSingleTagMeasurement(AprilVisionIO.SingleTagObservation observation) {
        int tagIndex = observation.id() - 1;

        // Skip if measurement is not the most recent
        if (singleTagPoses[tagIndex] != null && singleTagPoses[tagIndex].timestamp() >= observation.timestampSecs()) {
            return;
        }

        Optional<Pose2d> poseBufferSample = fusedPoseBuffer.getSample(observation.timestampSecs());
        if (poseBufferSample.isEmpty()) {
            return;
        }
        Rotation2d robotRotation = poseBufferSample.get().getRotation();

        Pose3d robotToCameraPose = Pose3d.kZero.transformBy(observation.cameraTransform());

        Translation2d cameraToTagTranslation = new Pose3d(
                Translation3d.kZero,
                new Rotation3d(
                        0.0,
                        -observation.pitch().getRadians(),
                        -observation.yaw().getRadians()))
                .transformBy(new Transform3d(observation.distanceMeters(), 0, 0, Rotation3d.kZero))
                .getTranslation()
                .rotateBy(new Rotation3d(
                        0, observation.cameraTransform().getRotation().getY(), 0))
                .toTranslation2d();
        Rotation2d cameraToTagRotation =
                robotRotation.plus(robotToCameraPose.toPose2d().getRotation().plus(cameraToTagTranslation.getAngle()));
        Optional<Pose3d> tagPose = FieldConstants.aprilTagLayout.getTagPose(observation.id());
        if (tagPose.isEmpty()) return;
        Translation2d fieldToCameraTranslation = new Pose2d(
                tagPose.get().toPose2d().getTranslation(), cameraToTagRotation.plus(Rotation2d.k180deg))
                .transformBy(new Transform2d(cameraToTagTranslation.getNorm(), 0.0, Rotation2d.kZero))
                .getTranslation();
        Pose2d robotPose = new Pose2d(
                fieldToCameraTranslation,
                robotRotation.plus(robotToCameraPose.toPose2d().getRotation()))
                .transformBy(new Transform2d(robotToCameraPose.toPose2d(), Pose2d.kZero));
        robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);

        singleTagPoses[tagIndex] =
                new SingleTagPoseEstimate(robotPose, observation.distanceMeters(), observation.timestampSecs());
    }

    public record SingleTagPoseEstimate(Pose2d pose, double distance, double timestamp) {}


}
