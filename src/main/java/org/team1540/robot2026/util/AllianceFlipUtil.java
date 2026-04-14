package org.team1540.robot2026.util;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;

public class AllianceFlipUtil {
    public static boolean shouldFlip() {
        return DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
    }

    public static Pose2d apply(Pose2d pose) {
        if (shouldFlip()) return FlippingUtil.flipFieldPose(pose);
        return pose;
    }

    public static List<Pose2d> apply(List<Pose2d> poses) {
        if (shouldFlip()) return poses.stream().map(AllianceFlipUtil::apply).toList();
        return poses;
    }

    public static Translation2d apply(Translation2d translation) {
        if (shouldFlip()) return FlippingUtil.flipFieldPosition(translation);
        return translation;
    }

    public static Rotation2d apply(Rotation2d rotation) {
        if (shouldFlip()) return FlippingUtil.flipFieldRotation(rotation);
        return rotation;
    }

    public static Trajectory<SwerveSample> apply(Trajectory<SwerveSample> trajectory) {
        if (shouldFlip()) return trajectory.flipped();
        return trajectory;
    }
}
