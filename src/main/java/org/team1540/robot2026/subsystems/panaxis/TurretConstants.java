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
    public static final int MAIN_CANCODER_ID = 2;
    public static final int SECONDARY_CANCODER_ID = 3;

    public static final double POS_ERR_TOLERANCE_DEGREES = 1;

    public static final double MAGNET_SENSOR_OFFSET = 1;











    public static double ENCODER_RANGE = 4096.0;  // 12-bit precision
    public static double GEAR_DIAMETRAL_PITCH = 10.0;    // Ratio of number of teeth to pitch diameter, that is a 10-tooth
    // Compute the maximum number of rotations (+1) for each planetary gear for a full rotation of
    // the driven gear
    public static double ROTATIONS_OF_GEAR_1_PER_DRIVEN_GEAR_ROTATION = (DRIVEN_GEAR_TOOTH_COUNT/PLANETARY_GEAR_1_TOOTH_COUNT) + 1;
    public static double ROTATIONS_OF_GEAR_2_PER_DRIVEN_GEAR_ROTATION = (DRIVEN_GEAR_TOOTH_COUNT/PLANETARY_GEAR_2_TOOTH_COUNT) + 1;

    // Compute the pitch diameters for the various gears
    public static double DrivenGearDiameter = ((double) (DRIVEN_GEAR_TOOTH_COUNT)) / GEAR_DIAMETRAL_PITCH;
    public static double PlanetaryGear1PitchDiameter = ((double) (PLANETARY_GEAR_1_TOOTH_COUNT)) / GEAR_DIAMETRAL_PITCH;
    public static double PlanetaryGear2PitchDiameter = ((double) (PLANETARY_GEAR_2_TOOTH_COUNT)) / GEAR_DIAMETRAL_PITCH;
    public static double DrivenGearDistancePerDegree = DrivenGearDiameter / 360.0;
    public static double DrivenGearDegreePerInch = 1.0 / DrivenGearDistancePerDegree;

    // Compute the encoder bit per unit distance for each planetary gear
    public static double PlanetaryGear1BitPerInch = ENCODER_RANGE / PlanetaryGear1PitchDiameter;
    public static double PlanetaryGear2BitPerInch = ENCODER_RANGE / PlanetaryGear2PitchDiameter;
    public static double PlanetaryGear1InchPerBit = 1.0 / PlanetaryGear1BitPerInch;
    public static  double PlanetaryGear2InchPerBit = 1.0 / PlanetaryGear2BitPerInch;
}
