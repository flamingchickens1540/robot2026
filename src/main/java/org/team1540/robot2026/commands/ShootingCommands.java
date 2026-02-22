package org.team1540.robot2026.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.FieldConstants;
import org.team1540.robot2026.RobotState;
import org.team1540.robot2026.subsystems.hood.Hood;
import org.team1540.robot2026.subsystems.shooter.Shooter;
import org.team1540.robot2026.subsystems.turret.Turret;
import org.team1540.robot2026.subsystems.turret.TurretConstants;
import org.team1540.robot2026.util.AllianceFlipUtil;
import org.team1540.robot2026.util.LoggedTunableNumber;

public class ShootingCommands {
    private static final Translation2d SHUFFLE_TARGET = FieldConstants.Tower.centerPoint;

    private static final LoggedTunableNumber shooterRPM = new LoggedTunableNumber("ShooterTuning/ShooterRPM", 1000);
    private static final LoggedTunableNumber hoodDegrees = new LoggedTunableNumber("ShooterTuning/HoodDegrees", 15);

    public static Command tuneShooterCommand(Turret turret, Shooter shooter, Hood hood) {
        return Commands.parallel(
                turret.commandToSetpoint(
                        () -> RobotState.getInstance().getHubAimingParameters().turretAngle(), () -> 0.0, true),
                shooter.commandVelocity(shooterRPM),
                hood.setpointCommand(() -> Rotation2d.fromDegrees(hoodDegrees.get())),
                Commands.run(() -> Logger.recordOutput(
                        "ShooterTuning/HubDistanceMeters",
                        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                                .minus(RobotState.getInstance()
                                        .getEstimatedPose()
                                        .transformBy(TurretConstants.ROBOT_TO_TURRET_2D)
                                        .getTranslation())
                                .getNorm())));
    }

    public static Command hubAimCommand(Turret turret, Shooter shooter, Hood hood) {
        return Commands.parallel(
                turret.commandToSetpoint(
                        () -> RobotState.getInstance().getHubAimingParameters().turretAngle(),
                        () -> RobotState.getInstance().getHubAimingParameters().turretVelocityRadPerSec(),
                        true),
                shooter.commandVelocity(
                        () -> RobotState.getInstance().getHubAimingParameters().shooterRPM()),
                hood.setpointCommand(
                        () -> RobotState.getInstance().getHubAimingParameters().hoodAngle()));
    }

    public static Command hubSOTMAimCommand(Turret turret, Shooter shooter, Hood hood) {
        return Commands.parallel(
                turret.commandToSetpoint(
                        () -> RobotState.getInstance().getHubSOTMAimingParameters().turretAngle(),
                        () -> RobotState.getInstance().getHubSOTMAimingParameters().turretVelocityRadPerSec(),
                        true),
                shooter.commandVelocity(
                        () -> RobotState.getInstance().getHubSOTMAimingParameters().shooterRPM()),
                hood.setpointCommand(
                        () -> RobotState.getInstance().getHubSOTMAimingParameters().hoodAngle()));
    }

    public static Command highShuffleAimCommand(Turret turret, Shooter shooter, Hood hood) {
        return Commands.parallel(
                turret.commandToSetpoint(
                        () -> RobotState.getInstance()
                                .getHighShuffleAimingParameters(AllianceFlipUtil.apply(SHUFFLE_TARGET))
                                .turretAngle(),
                        () -> RobotState.getInstance()
                                .getHighShuffleAimingParameters(AllianceFlipUtil.apply(SHUFFLE_TARGET))
                                .turretVelocityRadPerSec(),
                        true),
                shooter.commandVelocity(() -> RobotState.getInstance()
                        .getHighShuffleAimingParameters(AllianceFlipUtil.apply(SHUFFLE_TARGET))
                        .shooterRPM()),
                hood.setpointCommand(() -> RobotState.getInstance()
                        .getHighShuffleAimingParameters(AllianceFlipUtil.apply(SHUFFLE_TARGET))
                        .hoodAngle()));
    }
}
