package org.team1540.robot2026.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class TurretConstants {
    public static final int DRIVEN_GEAR_TOOTH_COUNT = 85;
    public static final int SMALL_ENCODER_GEAR_TOOTH_COUNT = 13;
    public static final int BIG_ENCODER_GEAR_TOOTH_COUNT = 14;
    public static final int POSSIBLE_POS_ACC_DIGITS = 12;

    public static final double GEAR_RATIO = 8.5 * 50 / 12;

    public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRadians(5.021413921812452);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(615 - ANGLE_OFFSET.getDegrees());
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(15 - ANGLE_OFFSET.getDegrees());

    // Tuning
    public static final double KS = 0.284;
    public static final double KV = 4.434;
    public static final double KA = 0.00;

    public static final double KP = 118;
    public static final double KI = 0;
    public static final double KD = 0;

    // IDS
    public static final int DRIVE_ID = 15;
    public static final int SMALL_ENCODER_CANCODER_ID = 21;
    public static final int BIG_ENCODER_CANCODER_ID = 22;

    public static final double POS_ERR_TOLERANCE_DEGREES = 10;

    public static final double SMALL_ENCODER_MAGNET_SENSOR_OFFSET = -0.740966796875;
    public static final double BIG_ENCODER_MAGNET_SENSOR_OFFSET = -0.91943359375;

    public static final double CRUISE_VELOCITY_RPS = 1; // was 3
    public static final double MAX_ACCEL_RPS2 = 2; // was 7

    public static final Transform2d ROBOT_TO_TURRET_2D =
            new Transform2d(Units.inchesToMeters(-7.5), 0.0, Rotation2d.kZero);
    public static final Transform3d ROBOT_TO_TURRET_3D = new Transform3d(
            ROBOT_TO_TURRET_2D.getX(), ROBOT_TO_TURRET_2D.getY(), Units.inchesToMeters(11.625), Rotation3d.kZero);
}
