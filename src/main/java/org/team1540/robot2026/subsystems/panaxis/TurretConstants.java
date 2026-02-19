package org.team1540.robot2026.subsystems.panaxis;

public class TurretConstants {

    public static final int  MAX_TURRET_ROTATION = 630;
    public static int DRIVEN_GEAR_TOOTH_COUNT = 85;
    public static int PLANETARY_GEAR_1_TOOTH_COUNT = 14;
    public static int PLANETARY_GEAR_2_TOOTH_COUNT = 13;
    public static final int DRIVE_GEAR_TEETH_COUNT = 10;
    public static final int POSSIBLE_POS_ACC_DIGITS = 14;

     public static final double DRIVEN_TO_DRIVE_RATIO = (double) DRIVEN_GEAR_TOOTH_COUNT / DRIVE_GEAR_TEETH_COUNT;

    // Tuning
    public static final double KS = 0.185;
    public static final double KV = 4.337;
    public static final double KA = 0.00;

    public static final double KP = 150;
    public static final double KI = 0;
    public static final double KD = 0;

    // IDS
    public static final int DRIVE_ID = 1;
    public static final int MAIN_CANCODER_ID = 21;
    public static final int SECONDARY_CANCODER_ID = 22;

    public static final double POS_ERR_TOLERANCE_DEGREES = 1;

    public static final double MAGNET_SENSOR_OFFSET = 1;


    public static final double CRUISE_VELOCITY_MPS = 3.0;
    public static final double MAXIMUM_ACCELERATION_MPS2 = 7.0;











}
