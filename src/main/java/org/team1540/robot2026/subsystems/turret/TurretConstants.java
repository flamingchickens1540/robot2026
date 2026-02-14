package org.team1540.robot2026.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretConstants {
    public static final int MOTOR_ID = 1;
    public static final int ENCODER_1_ID = 2;
    public static final int ENCODER_2_ID = 3;

    public static final double ENCODER_1_OFFSET_ROTS = 0.0;
    public static final double ENCODER_2_OFFSET_ROTS = 0.0;

    public static final double GEAR_RATIO = 67.0;
    public static final int MAIN_GEAR_TEETH = 85;
    public static final int ENCODER_1_TEETH = 13;
    public static final int ENCODER_2_TEETH = 14;

    public static final double POS_ERR_TOLERANCE_DEGREES = 1;

    public static final Rotation2d MIN_ANGLE = Rotation2d.kZero;
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(630);

    public static final double KP = 150;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0.185;
    public static final double KV = 4.337;

    public static final double CRUISE_VELOCITY_RPS = 6.7;
    public static final double MAX_ACCELERATION_RPS2 = 67.0;
    public static final double JERK_RPS3 = 670.0;
}
