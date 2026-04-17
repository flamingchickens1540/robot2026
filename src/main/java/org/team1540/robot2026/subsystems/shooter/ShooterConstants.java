package org.team1540.robot2026.subsystems.shooter;

public class ShooterConstants {
    public static final int RIGHT_ID = 7;
    public static final int LEFT_ID = 6;

    public static final double GEAR_RATIO = 1.0;

    public static final double VOLTAGE_KP = 0.3;
    public static final double VOLTAGE_KI = 0.0;
    public static final double VOLTAGE_KD = 0.0;
    public static final double VOLTAGE_KS = 0.30890533875168136;
    public static final double VOLTAGE_KV = 0.12009058138607204;

    public static final double TORQUE_KP = 6767.67; // Bang-bang torque control
    public static final double TORQUE_KI = 0.0;
    public static final double TORQUE_KD = 0.0;
    public static final double TORQUE_KS = 0.0;
    public static final double TORQUE_KV = 0.0;

    public static final boolean TORQUE_CONTROL = false;

    public static final double ERROR_TOLERANCE_RPM = 100;

    public static final double MOI_KGM2 = 0.0015;
}
