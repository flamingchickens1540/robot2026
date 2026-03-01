package org.team1540.robot2026.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
    public static int PIVOT_MOTOR_ID = 27;
    public static int INTAKE_MOTOR_ID = 14;

    public static double PIVOT_KS = 0.0;
    public static double PIVOT_KV = 4.76;
    public static double PIVOT_KG = 0.27;
    public static double PIVOT_KP = 670.0;
    public static double PIVOT_KI = 0.0;
    public static double PIVOT_KD = 0.67;

    public static double PIVOT_CRUISE_VELOCITY_RPS = 2.0;
    public static double PIVOT_ACCELERATION_RPS2 = 8.0;

    public static double SPIN_GEAR_RATIO = 1.0;
    public static double PIVOT_GEAR_RATIO = 48.75;

    public static Rotation2d PIVOT_MIN_ANGLE = Rotation2d.fromDegrees(-120.0);
    public static Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(0.0);

    public static double PIVOT_MOI_KGM2 = 0.2089;
}
