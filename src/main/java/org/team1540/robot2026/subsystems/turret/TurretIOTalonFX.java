package org.team1540.robot2026.subsystems.turret;

import static org.team1540.robot2026.subsystems.turret.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;

public class TurretIOTalonFX implements TurretIO {
    // Motion Magic
    private final MotionMagicVoltage profiledPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

    //  Motor
    private final TalonFX motor = new TalonFX(DRIVE_ID);
    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Angle> position = motor.getPosition();
    private final StatusSignal<Voltage> appliedVoltage = motor.getMotorVoltage();
    private final StatusSignal<Current> motorSupplyCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> motorTemp = motor.getDeviceTemp();
    private final StatusSignal<Current> motorStatorCurrent = motor.getStatorCurrent();

    // Encoders
    private final CANcoder smallCANcoder = new CANcoder(SMALL_ENCODER_CANCODER_ID);
    private final CANcoder bigCANcoder = new CANcoder(BIG_ENCODER_CANCODER_ID);
    private final StatusSignal<Angle> smallCANcoderPosition = smallCANcoder.getAbsolutePosition();
    private final StatusSignal<Angle> bigCANcoderPosition = bigCANcoder.getAbsolutePosition();

    public TurretIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.getRotations() + Units.degreesToRotations(5);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.getRotations() - Units.degreesToRotations(5);

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.5;

        config.Slot0.kP = KP;
        config.Slot0.kI = KI;
        config.Slot0.kD = KD;

        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;

        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCEL_RPS2;

        motor.getConfigurator().apply(config);

        CANcoderConfiguration configEncoder = new CANcoderConfiguration();
        configEncoder.MagnetSensor.MagnetOffset = SMALL_ENCODER_MAGNET_SENSOR_OFFSET;
        configEncoder.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        configEncoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        smallCANcoder.getConfigurator().apply(configEncoder);
        configEncoder.MagnetSensor.MagnetOffset = BIG_ENCODER_MAGNET_SENSOR_OFFSET;
        bigCANcoder.getConfigurator().apply(configEncoder);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                position,
                velocity,
                appliedVoltage,
                motorSupplyCurrent,
                motorTemp,
                motorStatorCurrent,
                smallCANcoderPosition,
                bigCANcoderPosition);
        motor.optimizeBusUtilization();
    }

    public void updateInputs(TurretIO.TurretIOInputs inputs) {
        StatusCode Status = BaseStatusSignal.refreshAll(
                position, velocity,
                appliedVoltage, motorSupplyCurrent,
                motorTemp, motorStatorCurrent,
                smallCANcoderPosition, bigCANcoderPosition);

        inputs.connected = Status.isOK();

        inputs.position = Rotation2d.fromRotations(position.getValueAsDouble());
        inputs.positionTimestamp =
                Timer.getFPGATimestamp() - position.getTimestamp().getLatency();
        inputs.velocityRPS = velocity.getValueAsDouble();

        inputs.supplyCurrentAmps = motorSupplyCurrent.getValueAsDouble();
        inputs.appliedVolts = appliedVoltage.getValueAsDouble();
        inputs.tempCelsius = motorTemp.getValueAsDouble();
        inputs.statorCurrentAmps = motorStatorCurrent.getValueAsDouble();

        inputs.smallEncoderConnected = smallCANcoder.isConnected();
        inputs.bigEncoderConnected = smallCANcoder.isConnected();
        inputs.smallEncoderPosition =
                Rotation2d.fromRotations(smallCANcoder.getAbsolutePosition().getValueAsDouble());
        inputs.bigEncoderPosition =
                Rotation2d.fromRotations(bigCANcoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setSetpoint(Rotation2d position, double voltageFF) {
        motor.setControl(
                profiledPositionControl.withPosition(position.getRotations()).withFeedForward(voltageFF));
    }

    @Override
    public void setMotorPosition(Rotation2d position) {
        motor.setPosition(position.getRotations());
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        motor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void configPID(double kP, double kI, double kD) {
        Slot0Configs configs = new Slot0Configs();
        motor.getConfigurator().refresh(configs);
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        motor.getConfigurator().apply(configs);
    }

    @Override
    public void configFF(double kS, double kV) {
        Slot0Configs configs = new Slot0Configs();
        motor.getConfigurator().refresh(configs);
        configs.kS = kS;
        configs.kV = kV;
        motor.getConfigurator().apply(configs);
    }
}
