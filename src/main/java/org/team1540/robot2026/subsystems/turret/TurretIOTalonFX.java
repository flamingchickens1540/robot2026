package org.team1540.robot2026.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;
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
import edu.wpi.first.units.measure.*;

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
    private final CANcoder gear1CANcoder = new CANcoder(GEAR_1_CANCODER_ID);
    private final CANcoder gear2CANcoder = new CANcoder(GEAR_2_CANCODER_ID);
    private final StatusSignal<Angle> gear1CANcoderPosition = gear1CANcoder.getAbsolutePosition();
    private final StatusSignal<Angle> gear2CANcoderPosition = gear2CANcoder.getAbsolutePosition();

    public TurretIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.SensorToMechanismRatio = DRIVEN_TO_DRIVE_RATIO;
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

        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_MPS;
        config.MotionMagic.MotionMagicAcceleration = MAXIMUM_ACCELERATION_MPS2;

        motor.getConfigurator().apply(config);

        CANcoderConfiguration configEncoder = new CANcoderConfiguration();
        configEncoder.MagnetSensor.MagnetOffset = GEAR_1_MAGNET_SENSOR_OFFSET;
        configEncoder.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        configEncoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        gear1CANcoder.getConfigurator().apply(configEncoder);
        configEncoder.MagnetSensor.MagnetOffset = GEAR_2_MAGNET_SENSOR_OFFSET;
        gear2CANcoder.getConfigurator().apply(configEncoder);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                position,
                velocity,
                appliedVoltage,
                motorSupplyCurrent,
                motorTemp,
                motorStatorCurrent,
                gear1CANcoderPosition,
                gear2CANcoderPosition);
        motor.optimizeBusUtilization();
    }

    public void updateInputs(TurretIO.TurretIOInputs inputs) {
        StatusCode Status = BaseStatusSignal.refreshAll(
                position, velocity,
                appliedVoltage, motorSupplyCurrent,
                motorTemp, motorStatorCurrent,
                gear1CANcoderPosition, gear2CANcoderPosition);

        inputs.connected = Status.isOK();
        inputs.supplyCurrentAmps = motorSupplyCurrent.getValueAsDouble();
        inputs.appliedVolts = appliedVoltage.getValueAsDouble();
        inputs.tempCelsius = motorTemp.getValueAsDouble();
        inputs.statorCurrentAmps = motorStatorCurrent.getValueAsDouble();
        inputs.position.fromDegrees(position.getValueAsDouble());
        inputs.velocityRadPerSec = velocity.getValueAsDouble();

        inputs.gear1EncoderConnected = gear1CANcoder.isConnected();
        inputs.gear2EncoderConnected = gear1CANcoder.isConnected();
        inputs.gear1EncoderPosition =
                Rotation2d.fromRotations(gear1CANcoder.getAbsolutePosition().getValueAsDouble());
        inputs.gear2EncoderPosition =
                Rotation2d.fromRotations(gear2CANcoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setSetpoint(Rotation2d position) {
        motor.setControl(profiledPositionControl.withPosition(Rotations.of(position.getDegrees())));
    }

    @Override
    public void setMotorPosition(Rotation2d position) {
        motor.setPosition(Rotations.of(position.getDegrees()));
    }

    @Override
    public double getGear1EncoderPos() {
        return gear1CANcoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    public double getGear2EncoderPos() {
        return gear2CANcoder.getAbsolutePosition().getValueAsDouble();
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
