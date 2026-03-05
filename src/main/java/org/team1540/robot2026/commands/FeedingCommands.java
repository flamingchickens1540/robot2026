package org.team1540.robot2026.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2026.FieldConstants;
import org.team1540.robot2026.RobotState;
import org.team1540.robot2026.subsystems.hood.Hood;
import org.team1540.robot2026.subsystems.spindexer.Spindexer;
import org.team1540.robot2026.subsystems.turret.Turret;

public class FeedingCommands {
    public static boolean shouldFeed(Turret turret, Hood hood) {
        // TODO better logic
        return !FieldConstants.Regions.behindOpposingHub.contains(RobotState.getInstance().getEstimatedPose().getTranslation())
                && !FieldConstants.Regions.underAllianceTower.contains(RobotState.getInstance().getEstimatedPose().getTranslation())
                && turret.atSetpoint(Rotation2d.fromDegrees(30.0))
                && hood.atSetpoint();
    }

    public static Command feedCommand(Turret turret, Hood hood, Spindexer spindexer) {
        return spindexer
                .runCommand(() -> shouldFeed(turret, hood) ? 1.0 : 0.0, () -> shouldFeed(turret, hood) ? 1.0 : 0.0)
                .withName("FeedShooterCommand");
    }
}
