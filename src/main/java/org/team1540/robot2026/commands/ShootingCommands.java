package org.team1540.robot2026.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.FieldConstants;
import org.team1540.robot2026.RobotState;
import org.team1540.robot2026.subsystems.drive.Drivetrain;
import org.team1540.robot2026.subsystems.hood.Hood;
import org.team1540.robot2026.subsystems.hood.HoodConstants;
import org.team1540.robot2026.subsystems.shooter.Shooter;
import org.team1540.robot2026.subsystems.turret.Turret;
import org.team1540.robot2026.subsystems.turret.TurretConstants;
import org.team1540.robot2026.util.AllianceFlipUtil;
import org.team1540.robot2026.util.LoggedTunableNumber;

public class ShootingCommands {

    private static final LoggedTunableNumber shooterRPM = new LoggedTunableNumber("ShooterTuning/ShooterRPM", 1000);
    private static final LoggedTunableNumber hoodDegrees = new LoggedTunableNumber("ShooterTuning/HoodDegrees", 15);

    public static Command tuneShooterCommand(Turret turret, Shooter shooter, Hood hood) {
        return Commands.parallel(
                turret.commandToSetpoint(
                        () -> RobotState.getInstance().getHubAimingParameters().turretAngle(), () -> 0.0, true),
                shooter.commandVelocity(shooterRPM),
                hood.setpointCommand(() -> Rotation2d.fromDegrees(hoodDegrees.get())),
                Commands.run(() -> Logger.recordOutput(
                        "ShooterTuning/TargetDistanceMeters",
                        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                                .minus(RobotState.getInstance()
                                        .getEstimatedPose()
                                        .transformBy(TurretConstants.ROBOT_TO_TURRET_2D)
                                        .getTranslation())
                                .getNorm())));
    }

    public static Command shooterAimCommand(Turret turret, Shooter shooter, Hood hood) {
        return Commands.parallel(
                        turret.commandToSetpoint(
                                () -> RobotState.getInstance()
                                        .getAimingParameters()
                                        .turretAngle(),
                                () -> RobotState.getInstance()
                                        .getAimingParameters()
                                        .turretVelocityRadPerSec(),
                                true),
                        shooter.commandVelocity(() ->
                                RobotState.getInstance().getAimingParameters().shooterRPM()),
                        hood.setpointCommand(() ->
                                RobotState.getInstance().getAimingParameters().hoodAngle()))
                .finallyDo(() -> hood.setSetpoint(HoodConstants.MIN_ANGLE));
    }

    public static Command shooterAimTurretLockedCommand(
            DoubleSupplier xPercent,
            DoubleSupplier yPercent,
            DoubleSupplier omegaPercent,
            Drivetrain drivetrain,
            Shooter shooter,
            Hood hood) {
        return Commands.deadline(
                        drivetrain.teleopDriveWithHeadingCommand(
                                xPercent,
                                yPercent,
                                omegaPercent,
                                () -> RobotState.getInstance().getTargetingMode() == RobotState.TargetingMode.HUB,
                                () -> RobotState.getInstance()
                                        .getAimingParameters()
                                        .turretAngle()
                                        .minus(RobotState.getInstance().getTurretAngle())),
                        shooter.commandVelocity(() ->
                                RobotState.getInstance().getAimingParameters().shooterRPM()),
                        hood.setpointCommand(() ->
                                RobotState.getInstance().getAimingParameters().hoodAngle()))
                .finallyDo(() -> hood.setSetpoint(HoodConstants.MIN_ANGLE));
    }

    public static Command hubAimCommand(Turret turret, Shooter shooter, Hood hood) {
        return Commands.parallel(
                        turret.commandToSetpoint(
                                () -> RobotState.getInstance()
                                        .getHubAimingParameters()
                                        .turretAngle(),
                                () -> RobotState.getInstance()
                                        .getHubAimingParameters()
                                        .turretVelocityRadPerSec(),
                                true),
                        shooter.commandVelocity(() -> RobotState.getInstance()
                                .getHubAimingParameters()
                                .shooterRPM()),
                        hood.setpointCommand(() -> RobotState.getInstance()
                                .getHubAimingParameters()
                                .hoodAngle()))
                .finallyDo(() -> hood.setSetpoint(HoodConstants.MIN_ANGLE));
    }

    public static Command hubAimTurretLockedCommand(
            DoubleSupplier xPercent,
            DoubleSupplier yPercent,
            DoubleSupplier omegaPercent,
            Drivetrain drivetrain,
            Shooter shooter,
            Hood hood,
            Turret turret) {
        return Commands.parallel(
                        drivetrain.teleopDriveWithHeadingCommand(
                                xPercent,
                                yPercent,
                                omegaPercent,
                                () -> RobotState.getInstance().getTargetingMode() == RobotState.TargetingMode.HUB,
                                () -> RobotState.getInstance()
                                        .getHubAimingParameters()
                                        .turretAngle()
                                        .minus(turret.getPosition())),
                        shooter.commandVelocity(() -> RobotState.getInstance()
                                .getHubAimingParameters()
                                .shooterRPM()),
                        hood.setpointCommand(() -> RobotState.getInstance()
                                .getHubAimingParameters()
                                .hoodAngle()))
                .finallyDo(() -> hood.setSetpoint(HoodConstants.MIN_ANGLE));
    }

    public static Command closeShotCommand(Shooter shooter, Hood hood) {
        return Commands.parallel(
                        shooter.commandVelocity(() -> 1678.0), hood.setpointCommand(() -> Rotation2d.fromDegrees(15.0)))
                .finallyDo(() -> hood.setSetpoint(HoodConstants.MIN_ANGLE));
    }

    public static Command trenchShotCommand(Shooter shooter, Hood hood) {
        return Commands.parallel(
                        shooter.commandVelocity(() -> 2056.0), hood.setpointCommand(() -> Rotation2d.fromDegrees(23.4)))
                .finallyDo(() -> hood.setSetpoint(HoodConstants.MIN_ANGLE));
    }

    public static Command shuffleAimCommand(Turret turret, Shooter shooter, Hood hood) {
        return Commands.parallel(
                        turret.commandToSetpoint(
                                () -> RobotState.getInstance()
                                        .getShuffleAimingParameters()
                                        .turretAngle(),
                                () -> RobotState.getInstance()
                                        .getShuffleAimingParameters()
                                        .turretVelocityRadPerSec(),
                                true),
                        shooter.commandVelocity(() -> RobotState.getInstance()
                                .getShuffleAimingParameters()
                                .shooterRPM()),
                        hood.setpointCommand(() -> RobotState.getInstance()
                                .getShuffleAimingParameters()
                                .hoodAngle()))
                .finallyDo(() -> hood.setSetpoint(HoodConstants.MIN_ANGLE));
    }

    public static Command shuffleAimTurretLockedCommand(
            DoubleSupplier xPercent,
            DoubleSupplier yPercent,
            DoubleSupplier omegaPercent,
            Drivetrain drivetrain,
            Shooter shooter,
            Hood hood,
            Turret turret) {
        return Commands.parallel(
                        drivetrain.teleopDriveWithHeadingCommand(
                                xPercent,
                                yPercent,
                                omegaPercent,
                                () -> RobotState.getInstance().getTargetingMode() == RobotState.TargetingMode.HUB,
                                () -> (RobotState.getInstance()
                                                .getShuffleAimingParameters()
                                                .turretAngle())
                                        .minus(turret.getPosition())),
                        shooter.commandVelocity(() -> RobotState.getInstance()
                                .getShuffleAimingParameters()
                                .shooterRPM()),
                        hood.setpointCommand(() -> RobotState.getInstance()
                                .getShuffleAimingParameters()
                                .hoodAngle()))
                .finallyDo(() -> hood.setSetpoint(HoodConstants.MIN_ANGLE));
    }
}
