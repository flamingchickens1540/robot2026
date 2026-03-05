package org.team1540.robot2026.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class AprilTagVisionConstants {
    public static final String TURRET_CAMERA_NAME = "turret-camera";
    public static final String BL_CAMERA_NAME = "back-left";
    public static final String BR_CAMERA_NAME = "back-right";

    public static final Transform3d TURRET_TO_CAMERA = new Transform3d(
            Units.inchesToMeters(6.429),
            0.0,
            Units.inchesToMeters(8.649),
            new Rotation3d(0.0, Math.toRadians(-28.0), 0.0));
    public static final Transform3d ROBOT_TO_BL_CAMERA = new Transform3d();
    public static final Transform3d ROBOT_TO_FL_CAMERA = new Transform3d();

    public static final double XY_STD_DEV_COEFF = 0.15;
    public static final double ROT_STD_DEV_COEFF = 0.25;

    public static final double MIN_ACCEPTED_NUM_TAGS = 1;
    public static final double MAX_AMBIGUITY = 0.2;
    public static final double MAX_OUTSIDE_OF_FIELD_TOLERANCE = 0.1;
    public static final double MAX_ROBOT_Z_TOLERANCE = 0.5;

    public static final int SIM_RES_WIDTH = 1280;
    public static final int SIM_RES_HEIGHT = 800;
    public static final Rotation2d SIM_DIAGONAL_FOV = Rotation2d.fromDegrees(70);
    public static final double SIM_FPS = 25.0;
    public static final double SIM_AVG_LATENCY_MS = 12.5;
}
