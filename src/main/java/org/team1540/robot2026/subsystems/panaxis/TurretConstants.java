package org.team1540.robot2026.subsystems.panaxis;

public class TurretConstants {
    public double MAX_TURRET_ROTATION = 320;
    public static final int SUN_GEAR_TEETH_COUNT = 85;
    public static final int DRIVE_GEAR_TEETH_COUNT = 10;
    public static final int EXTRA_GEAR_TEETH_COUNT = 13;
    public static final int EXTRA2_GEAR_TEETH_COUNT = 14;

    public static final double KS = 0.185;
    public static final double KV = 4.337;
    public static final double KA = 0.00;
    public static final double KP = 150;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KG = 0.425; // TODO update with correct value

    public static final int DRIVE_ID = 1;
    public static final int MAIN_CANCODER_ID = 2;
    public static final int SECONDARY_CANCODER_ID = 3;
    public static final double UPDATE_HRTZ = 50;
}
