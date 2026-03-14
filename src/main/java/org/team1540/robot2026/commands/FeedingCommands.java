package org.team1540.robot2026.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.FieldConstants;
import org.team1540.robot2026.RobotState;
import org.team1540.robot2026.subsystems.hood.Hood;
import org.team1540.robot2026.subsystems.spindexer.Spindexer;
import org.team1540.robot2026.subsystems.turret.Turret;
import org.team1540.robot2026.util.AllianceFlipUtil;

public class FeedingCommands {
    public static boolean shouldFeed(Turret turret, Hood hood, BooleanSupplier override) {
        return (!FieldConstants.Regions.behindOpposingHub.contains(AllianceFlipUtil.apply(
                                RobotState.getInstance().getTurretPose().getTranslation()))
                        && !FieldConstants.Regions.underAllianceTower.contains(AllianceFlipUtil.apply(
                                RobotState.getInstance().getTurretPose().getTranslation()))
                        && !RobotState.getInstance().shouldLowerHood()
                        && turret.atSetpoint(Rotation2d.fromDegrees(30.0))
                        && hood.atSetpoint())
                || override.getAsBoolean();
    }

    public static Command feedCommand(Turret turret, Hood hood, Spindexer spindexer) {
        return feedCommand(turret, hood, spindexer, () -> false);
    }

    public static Command feedCommand(Turret turret, Hood hood, Spindexer spindexer, BooleanSupplier override) {
        return spindexer
                .runCommand(
                        () -> shouldFeed(turret, hood, override) ? 1.0 : 0.0,
                        () -> shouldFeed(turret, hood, override) ? 1.0 : 0.0)
                .alongWith(Commands.run(() -> {
                    Logger.recordOutput("Spindexer/IsFeeding", shouldFeed(turret, hood, override));
                    Logger.recordOutput("Spindexer/FeedOverride", override.getAsBoolean());
                }))
                .withName("FeedShooterCommand");
    }
}
