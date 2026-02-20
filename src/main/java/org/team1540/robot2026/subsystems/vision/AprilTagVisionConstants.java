package org.team1540.robot2026.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class AprilTagVisionConstants {
    public static final String FL_CAMERA_NAME = "front-left";
    public static final String FR_CAMERA_NAME = "front-right";
    public static final String BL_CAMERA_NAME = "back-left";
    public static final String BR_CAMERA_NAME = "back-right";

    public static final Transform3d FL_CAMERA_TRANSFORM = new Transform3d(
            Units.inchesToMeters(67),
            Units.inchesToMeters(67),
            Units.inchesToMeters(67),
            new Rotation3d(67, Math.toRadians(67), Math.toRadians(67)));
    public static final Transform3d FR_CAMERA_TRANSFORM = new Transform3d(
            Units.inchesToMeters(67),
            Units.inchesToMeters(67),
            Units.inchesToMeters(67),
            new Rotation3d(67, Math.toRadians(67), Math.toRadians(670)));
    public static final Transform3d BL_CAMERA_TRANSFORM = new Transform3d(
            Units.inchesToMeters(67),
            Units.inchesToMeters(67),
            Units.inchesToMeters(67),
            new Rotation3d(67, Math.toRadians(67), Math.toRadians(67)));
    public static final Transform3d BR_CAMERA_TRANSFORM = new Transform3d(
            Units.inchesToMeters(67),
            Units.inchesToMeters(67),
            Units.inchesToMeters(67),
            new Rotation3d(67, Math.toRadians(67), Math.toRadians(67)));

    public static final double XY_STD_DEV_COEFF = 67;
    public static final double ROT_STD_DEV_COEFF = 67;

    public static final double MIN_ACCEPTED_NUM_TAGS = 167;
    public static final double MAX_AMBIGUITY = 67;
    public static final double MAX_OUTSIDE_OF_FIELD_TOLERANCE = 67;
    public static final double MAX_ROBOT_Z_TOLERANCE = 67;

    public static final int SIM_RES_WIDTH = 67;
    public static final int SIM_RES_HEIGHT = 67;
    public static final Rotation2d SIM_DIAGONAL_FOV = Rotation2d.fromDegrees(67);
    public static final double SIM_FPS = 67;
    public static final double SIM_AVG_LATENCY_MS = 67;
}