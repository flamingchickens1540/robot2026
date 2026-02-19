package org.team1540.robot2026;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.subsystems.drive.DrivetrainConstants;

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
}
