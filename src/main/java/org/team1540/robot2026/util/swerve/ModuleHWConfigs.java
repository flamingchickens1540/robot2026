package org.team1540.robot2026.util.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public record ModuleHWConfigs(
        TalonFXConfiguration driveConfig, TalonFXConfiguration turnConfig, CANcoderConfiguration turnEncoderConfig) {
    public static final Slot0Configs DRIVE_VOLTAGE_GAINS = new Slot0Configs()
            .withKP(0.1)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.25854004341546240)
            .withKV(0.9146790761644792);

    public static final Slot0Configs DRIVE_TORQUE_GAINS = new Slot0Configs()
            .withKP(35.0)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(5.0);

    public static final Slot0Configs STEER_GAINS = new Slot0Configs()
            .withKP(100)
            .withKI(0)
            .withKD(0.5)
            .withKS(0.1)
            .withKV(2.33)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    public static ModuleHWConfigs fromModuleConstants(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        TalonFXConfiguration driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;

        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        driveConfig.CurrentLimits.SupplyCurrentLimit = 70;
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.Slot0 = constants.DriveMotorClosedLoopOutput
                        == SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC
                ? DRIVE_TORQUE_GAINS
                : DRIVE_VOLTAGE_GAINS;

        TalonFXConfiguration turnConfig = constants.SteerMotorInitialConfigs;
        turnConfig.MotorOutput.Inverted = constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Feedback.FeedbackSensorSource = switch (constants.FeedbackSource) {
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
            case FusedCANdiPWM1 -> FeedbackSensorSourceValue.FusedCANdiPWM1;
            case FusedCANdiPWM2 -> FeedbackSensorSourceValue.FusedCANdiPWM2;
            case SyncCANdiPWM1 -> FeedbackSensorSourceValue.SyncCANdiPWM1;
            case SyncCANdiPWM2 -> FeedbackSensorSourceValue.SyncCANdiPWM2;
            case RemoteCANdiPWM1 -> FeedbackSensorSourceValue.RemoteCANdiPWM1;
            case RemoteCANdiPWM2 -> FeedbackSensorSourceValue.RemoteCANdiPWM2;
            default -> throw new IllegalStateException(
                    "Invalid turn feedback sensor source" + constants.FeedbackSource);};
        turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        turnConfig.Feedback.SensorToMechanismRatio = 1.0;
        turnConfig.CurrentLimits.SupplyCurrentLimit = 40;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.Slot0 = STEER_GAINS;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = Units.radiansToRotations(
                DCMotor.getFalcon500Foc(1).withReduction(constants.SteerMotorGearRatio).freeSpeedRadPerSec);
        turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.1;
        turnConfig.MotionMagic.MotionMagicExpo_kV = 1.0
                / Units.radiansToRotations(
                        DCMotor.getFalcon500Foc(1).withReduction(constants.SteerMotorGearRatio).KvRadPerSecPerVolt);
        turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        CANcoderConfiguration turnEncoderConfig = constants.EncoderInitialConfigs;
        turnEncoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        turnEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        turnEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        return new ModuleHWConfigs(driveConfig, turnConfig, turnEncoderConfig);
    }
}
