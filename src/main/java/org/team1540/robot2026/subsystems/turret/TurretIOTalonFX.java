package org.team1540.robot2026.subsystems.turret;

import static org.team1540.robot2026.subsystems.turret.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

public class TurretIOTalonFX implements TurretIO {
    private final TalonFX motor = new TalonFX(MOTOR_ID);
    private final CANcoder encoder1 = new CANcoder(SMALL_ENCODER_ID);
    private final CANcoder encoder2 = new CANcoder(BIG_ENCODER_ID);

    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private final StatusSignal<Angle> position = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
    private final StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent();
    private final StatusSignal<Temperature> temp = motor.getDeviceTemp();

    private final StatusSignal<Angle> encoder1Position = encoder1.getPosition();
    private final StatusSignal<Angle> encoder2Position = encoder2.getPosition();

    private final VoltageOut voltageCtrlReq = new VoltageOut(0).withEnableFOC(true);
    private final MotionMagicVoltage positionCtrlReq =
            new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);

    public TurretIOTalonFX() {
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motorConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.getRotations();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.getRotations();

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 80.0;

        motorConfig.Slot0.kP = KP;
        motorConfig.Slot0.kI = KI;
        motorConfig.Slot0.kD = KD;
        motorConfig.Slot0.kS = KS;
        motorConfig.Slot0.kV = KV;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
        motorConfig.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION_RPS2;
        motorConfig.MotionMagic.MotionMagicJerk = JERK_RPS3;

        motor.getConfigurator().apply(motorConfig);

        motor.setPosition(0);
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = ENCODER_1_OFFSET_ROTS;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        encoder1.getConfigurator().apply(encoderConfig);
        encoderConfig.MagnetSensor.MagnetOffset = ENCODER_2_OFFSET_ROTS;

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                voltage,
                supplyCurrent,
                statorCurrent,
                temp,
                encoder1Position,
                encoder2Position);

        motor.optimizeBusUtilization();
        encoder1.optimizeBusUtilization();
        encoder2.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                        position, velocity, voltage, supplyCurrent, statorCurrent, temp)
                .isOK();
        inputs.position = Rotation2d.fromRotations(position.getValueAsDouble());
        inputs.velocityRPS = velocity.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();

        inputs.encoder1Connected = encoder1Position.refresh().getStatus().isOK();
        inputs.encoder1Position = Rotation2d.fromRotations(encoder1Position.getValueAsDouble());

        inputs.encoder2Connected = encoder2Position.refresh().getStatus().isOK();
        inputs.encoder2Position = Rotation2d.fromRotations(encoder2Position.getValueAsDouble());
    }

    @Override
    public void setSetpoint(Rotation2d position) {
        motor.setControl(positionCtrlReq.withPosition(position.getRotations()));
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageCtrlReq.withOutput(volts));
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        motor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void configPID(double kP, double kI, double kD, double kS, double kV) {
        motorConfig.Slot0.kP = kP;
        motorConfig.Slot0.kI = kI;
        motorConfig.Slot0.kD = kD;
        motorConfig.Slot0.kS = kS;
        motorConfig.Slot0.kV = kV;
        motor.getConfigurator().apply(motorConfig);
    }
}
