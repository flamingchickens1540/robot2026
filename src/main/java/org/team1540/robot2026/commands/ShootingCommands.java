package org.team1540.robot2026.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import org.team1540.robot2026.util.hid.EnvisionController;

public class ShootingCommands {
    private static final double SHUFFLE_TARGET_X = FieldConstants.LinesVertical.starting - 2.0;

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
            EnvisionController controller, Drivetrain drivetrain, Shooter shooter, Hood hood, Turret turret) {
        return Commands.parallel(
                        drivetrain.teleopDriveWithHeadingCommand(controller, () -> RobotState.getInstance()
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

    public static Command shuffleAimCommand(Turret turret, Shooter shooter, Hood hood) {
        Translation2d shuffleTarget;
        if (AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose()).getY() < FieldConstants.LinesHorizontal.center) {
            shuffleTarget = new Translation2d(SHUFFLE_TARGET_X, FieldConstants.LinesHorizontal.rightBumpEnd);
        } else {
            shuffleTarget = new Translation2d(SHUFFLE_TARGET_X, FieldConstants.LinesHorizontal.leftBumpEnd);
        }
        return Commands.parallel(
                        turret.commandToSetpoint(
                                () -> RobotState.getInstance()
                                        .getShuffleAimingParameters(shuffleTarget)
                                        .turretAngle(),
                                () -> RobotState.getInstance()
                                        .getShuffleAimingParameters(AllianceFlipUtil.apply(shuffleTarget))
                                        .turretVelocityRadPerSec(),
                                true),
                        shooter.commandVelocity(() -> RobotState.getInstance()
                                .getShuffleAimingParameters(AllianceFlipUtil.apply(shuffleTarget))
                                .shooterRPM()),
                        hood.setpointCommand(() -> RobotState.getInstance()
                                .getShuffleAimingParameters(AllianceFlipUtil.apply(shuffleTarget))
                                .hoodAngle()))
                .finallyDo(() -> hood.setSetpoint(HoodConstants.MIN_ANGLE));
    }

    public static Command shuffleAimTurretLockedCommand(
            EnvisionController controller, Drivetrain drivetrain, Shooter shooter, Hood hood, Turret turret) {
        Translation2d shuffleTarget;
        if (AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose()).getY() < FieldConstants.LinesHorizontal.center) {
            shuffleTarget = new Translation2d(SHUFFLE_TARGET_X, FieldConstants.LinesHorizontal.rightBumpEnd);
        } else {
            shuffleTarget = new Translation2d(SHUFFLE_TARGET_X, FieldConstants.LinesHorizontal.leftBumpEnd);
        }
        return Commands.parallel(
                        drivetrain.teleopDriveWithHeadingCommand(controller, () -> (RobotState.getInstance()
                                .getShuffleAimingParameters(AllianceFlipUtil.apply(shuffleTarget))
                                .turretAngle())
                                .minus(turret.getPosition())),
                        shooter.commandVelocity(() -> RobotState.getInstance()
                                .getShuffleAimingParameters(AllianceFlipUtil.apply(shuffleTarget))
                                .shooterRPM()),
                        hood.setpointCommand(() -> RobotState.getInstance()
                                .getShuffleAimingParameters(AllianceFlipUtil.apply(shuffleTarget))
                                .hoodAngle()))
                .finallyDo(() -> hood.setSetpoint(HoodConstants.MIN_ANGLE));
    }
}