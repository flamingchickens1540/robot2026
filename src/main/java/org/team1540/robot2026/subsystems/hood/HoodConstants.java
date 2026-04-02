package org.team1540.robot2026.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodConstants {
    public static final int MOTOR_ID = 11;

    public static final double GEAR_RATIO = 60.0 / 12.0 * 154.0 / 10.0;
    public static final Rotation2d POSITION_ERR_TOLERANCE = Rotation2d.fromDegrees(5.0);

    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(15.0);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(45.0);

    public static final double ZERO_CURRENT_AMPS = 30;

    public static final double KP = 254.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KG = 0.0;

    public static final double CRUISE_VELOCITY_RPS = 1.5;
    public static final double MAX_ACCELERATION_RPS2 = 6.0;
    public static final double JERK_RPS3 = 670.0;

    public static final double LENGTH_METERS = Units.inchesToMeters(10.0);
    public static final double MASS_KG = Units.lbsToKilograms(3.767);
    public static final double MOI_KGM2 = SingleJointedArmSim.estimateMOI(LENGTH_METERS, MASS_KG);

    public static final Transform3d TURRET_TO_HOOD =
            new Transform3d(Units.inchesToMeters(4.25), 0.0, Units.inchesToMeters(5.875), Rotation3d.kZero);
}
