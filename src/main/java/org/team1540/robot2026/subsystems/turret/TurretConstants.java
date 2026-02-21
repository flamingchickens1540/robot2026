package org.team1540.robot2026.subsystems.turret;

public class TurretConstants {

    public static final int MAX_TURRET_ROTATION = 630;
    public static int DRIVEN_GEAR_TOOTH_COUNT = 85;
    public static int SMALL_ENCODER_GEAR_TOOTH_COUNT = 13;
    public static int BIG_ENCODER_GEAR_TOOTH_COUNT = 14;
    public static final int DRIVE_GEAR_TEETH_COUNT = 12; // dont know
    public static final int SECOND_GEAR_TEETH_COUNT = 12; // dont know
    public static final int POSSIBLE_POS_ACC_DIGITS = 12;

    public static final double GEAR_RATIO = 8.5 * 50 / 12;

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

    public static final double POS_ERR_TOLERANCE_DEGREES = 2;

    public static final double SMALL_ENCODER_MAGNET_SENSOR_OFFSET = -0.5021953125;
    public static final double BIG_ENCODER_MAGNET_SENSOR_OFFSET = -0.1240234375;

    public static final double CRUISE_VELOCITY_MPS = 0.1; // was 3
    public static final double MAXIMUM_ACCELERATION_MPS2 = 0.1; // was 7
}
