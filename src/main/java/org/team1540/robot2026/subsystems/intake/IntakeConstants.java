package org.team1540.robot2026.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
    public static int PIVOT_MOTOR_ID = 0;
    public static int INTAKE_MOTOR_ID = 0;

    public static double PIVOT_KS = 0.0;
    public static double PIVOT_KV = 0.0;
    public static double PIVOT_KG = 0.0;
    public static double PIVOT_KP = 0.0;
    public static double PIVOT_KI = 0.0;
    public static double PIVOT_KD = 0.0;

    public static double PIVOT_CRUISE_VELOCITY_RPS = 0.0;
    public static double PIVOT_ACCELERATION_RPS2 = 0.0;

    public static double SPIN_GEAR_RATIO = 0.0;
    public static double PIVOT_GEAR_RATIO = 0.0;

    public static Rotation2d PIVOT_MIN_ANGLE = Rotation2d.fromRotations(0.0);
    public static Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromRotations(0.0);

    public static double PIVOT_MOI = 0.0;
}
