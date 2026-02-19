package org.team1540.robot2026.subsystems.climber;

import edu.wpi.first.math.util.Units;

public class ClimberConstants {
    // CONSTANTS NEED TO BE UPDATED
    public static final int LEFT_MOTOR_ID = 0;
    public static final int RIGHT_MOTOR_ID = 0;
    public static final int UPPER_LIMIT_ID = 0;
    public static final int LOWER_LIMIT_ID = 0;

    public static final double MAX_HEIGHT_M = Units.inchesToMeters(0);
    public static final double MIN_HEIGHT_M = 0;
    public static final double POS_ERR_TOLERANCE_M = 0.01;
    public static final double SIM_CARRIAGE_MASS_KG = 1.55;
    public static final double SPROCKET_RADIUS_M = Units.inchesToMeters(1.504 / 2);

    public static final double GEAR_RATIO = 3.0;
    public static final double KS = 0.185;
    public static final double KV = 4.337;
    public static final double KA = 0.00;
    public static final double KP = 150;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KG = 0.425;

    public static final double CRUISE_VELOCITY_MPS = 3.0;
    public static final double ACCELERATION_MPS2 = 7.0;
}
