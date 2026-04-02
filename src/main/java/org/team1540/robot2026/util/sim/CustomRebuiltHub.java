package org.team1540.robot2026.util.sim;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.Arrays;
import java.util.Random;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;
import org.team1540.robot2026.FieldConstants;

public class CustomRebuiltHub extends RebuiltHub {
    // Fix red shoot pose flipping
    private static final Pose3d[] redShootPoses = Arrays.stream(blueShootPoses)
            .map(pose -> new Pose3d(
                    FieldConstants.fieldLength - pose.getX(),
                    FieldConstants.fieldWidth - pose.getY(),
                    pose.getZ(),
                    pose.getRotation().plus(new Rotation3d(Rotation2d.k180deg))))
            .toArray(Pose3d[]::new);

    private static final Random rng = new Random();

    public CustomRebuiltHub(Arena2026Rebuilt arena, boolean isBlue) {
        super(arena, isBlue);
    }

    @Override
    protected void addPoints() {
        arena.addValueToMatchBreakdown(isBlue, "TotalFuelInHub", 1);
        arena.addValueToMatchBreakdown(isBlue, "WastedFuel", arena.isActive(isBlue) ? 0 : 1);
        arena.addToScore(isBlue, arena.isActive(isBlue) ? 1 : 0);

        Pose3d shootPose = isBlue ? blueShootPoses[rng.nextInt(4)] : redShootPoses[rng.nextInt(4)];

        arena.addPieceWithVariance(
                shootPose.getTranslation().toTranslation2d(),
                new Rotation2d(shootPose.getRotation().getZ()),
                shootPose.getMeasureZ(),
                MetersPerSecond.of(2),
                shootPose.getRotation().getMeasureY(),
                0,
                0.02,
                15,
                0.2,
                5);
    }
}
