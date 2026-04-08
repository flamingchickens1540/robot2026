package org.team1540.robot2026;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.subsystems.hood.HoodConstants;
import org.team1540.robot2026.subsystems.intake.IntakeConstants;
import org.team1540.robot2026.subsystems.turret.TurretConstants;

public class MechanismVisualizer {
    private static Rotation2d turretAngle = Rotation2d.kZero;
    private static Rotation2d turretSetpoint = Rotation2d.kZero;

    private static Rotation2d hoodAngle = Rotation2d.kZero;
    private static Rotation2d hoodSetpoint = Rotation2d.kZero;

    private static Rotation2d intakeAngle = Rotation2d.kZero;
    private static Rotation2d intakeSetpoint = Rotation2d.kZero;

    public static void periodic() {
        Pose3d turretPose = Pose3d.kZero.transformBy(TurretConstants.ROBOT_TO_TURRET_3D.plus(
                new Transform3d(0, 0, 0, new Rotation3d(0, 0, turretAngle.getRadians()))));
        Pose3d turretSetpointPose = new Pose3d(
                TurretConstants.ROBOT_TO_TURRET_3D.getTranslation(), new Rotation3d(0, 0, turretSetpoint.getRadians()));

        Pose3d hoodPose = turretPose.transformBy(HoodConstants.TURRET_TO_HOOD.plus(
                new Transform3d(0, 0, 0, new Rotation3d(0, hoodAngle.getRadians(), 0))));
        Pose3d hoodSetpointPose = turretSetpointPose.transformBy(HoodConstants.TURRET_TO_HOOD.plus(
                new Transform3d(0, 0, 0, new Rotation3d(0, hoodSetpoint.getRadians(), 0))));

        Pose3d intakePose = Pose3d.kZero.transformBy(IntakeConstants.ROBOT_TO_PIVOT.plus(
                new Transform3d(0, 0, 0, new Rotation3d(0, intakeAngle.getRadians(), 0))));
        Pose3d intakeSetpointPose = new Pose3d(
                IntakeConstants.ROBOT_TO_PIVOT.getTranslation(), new Rotation3d(0, intakeSetpoint.getRadians(), 0));

        double hopperExtensionMeters = MathUtil.clamp(
                IntakeConstants.MAX_EXTENSION_METERS * intakeAngle.getCos(), 0, IntakeConstants.MAX_EXTENSION_METERS);
        double hopperExtensionSetpointMeters = MathUtil.clamp(
                IntakeConstants.MAX_EXTENSION_METERS * intakeSetpoint.getCos(),
                0,
                IntakeConstants.MAX_EXTENSION_METERS);
        Pose3d hopperPose = new Pose3d(hopperExtensionMeters, 0, 0, Rotation3d.kZero);
        Pose3d hopperSetpointPose = new Pose3d(hopperExtensionSetpointMeters, 0, 0, Rotation3d.kZero);

        Logger.recordOutput("Mechanism3d/Measured", turretPose, hoodPose, intakePose, hopperPose);
        Logger.recordOutput(
                "Mechanism3d/Setpoint", turretSetpointPose, hoodSetpointPose, intakeSetpointPose, hopperSetpointPose);
    }

    public static void addTurretData(Rotation2d angle, Rotation2d setpoint) {
        turretAngle = angle;
        turretSetpoint = setpoint;
    }

    public static void addHoodData(Rotation2d angle, Rotation2d setpoint) {
        hoodAngle = angle.minus(HoodConstants.MIN_ANGLE);
        hoodSetpoint = setpoint.minus(HoodConstants.MIN_ANGLE);
    }

    public static void addIntakeData(Rotation2d angle, Rotation2d setpoint) {
        intakeAngle = angle;
        intakeSetpoint = setpoint;
    }
}
