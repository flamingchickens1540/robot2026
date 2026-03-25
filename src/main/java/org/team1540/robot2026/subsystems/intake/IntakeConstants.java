package org.team1540.robot2026.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static int PIVOT_MOTOR_ID = 27;
    public static int LEFT_SPIN_MOTOR_ID = 14;
    public static int RIGHT_SPIN_MOTOR_ID = 21;

    public static double PIVOT_KS = 0.0;
    public static double PIVOT_KV = 4.76;
    public static double PIVOT_KG = 0.27;
    public static double PIVOT_KP = 670.0;
    public static double PIVOT_KI = 0.0;
    public static double PIVOT_KD = 0.67;

    public static double PIVOT_CRUISE_VELOCITY_RPS = 2.0;
    public static double PIVOT_ACCELERATION_RPS2 = 4.0;

    public static double SPIN_GEAR_RATIO = 20.0 / 15;
    public static double PIVOT_GEAR_RATIO = 48.75;

    public static Rotation2d PIVOT_MIN_ANGLE = Rotation2d.fromDegrees(-140.0);
    public static Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(0.0);
    public static Rotation2d PIVOT_JIGGLE_ANGLE = Rotation2d.fromDegrees(-30);
    public static Rotation2d PIVOT_DEPOT_ANGLE = Rotation2d.fromDegrees(-7);

    public static final double WIDTH_METERS = Units.inchesToMeters(22.0);
    public static final double MAX_EXTENSION_METERS = 0.269077;

    public static Transform3d ROBOT_TO_PIVOT =
            new Transform3d(Units.inchesToMeters(12.600), 0.0, Units.inchesToMeters(8.500), Rotation3d.kZero);
    public static double PIVOT_MOI_KGM2 = 0.2089;
}
