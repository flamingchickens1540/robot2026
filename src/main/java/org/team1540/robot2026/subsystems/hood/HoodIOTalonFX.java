package org.team1540.robot2026.subsystems.hood;

import static org.team1540.robot2026.subsystems.hood.HoodConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

public class HoodIOTalonFX implements HoodIO {
    private final TalonFX motor = new TalonFX(MOTOR_ID);
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private final StatusSignal<Angle> position = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
    private final StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent();
    private final StatusSignal<Temperature> temp = motor.getDeviceTemp();

    private final VoltageOut voltageCtrlReq = new VoltageOut(0).withEnableFOC(true);
    private final MotionMagicVoltage positionCtrlReq =
            new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);

    public HoodIOTalonFX() {

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motorConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        motorConfig.Slot0.kP = KP;
        motorConfig.Slot0.kI = KI;
        motorConfig.Slot0.kD = KD;
        motorConfig.Slot0.kS = KS;
        motorConfig.Slot0.kV = KV;
        motorConfig.Slot0.kG = KG;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
        motorConfig.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION_RPS2;
        motorConfig.MotionMagic.MotionMagicJerk = JERK_RPS3;

        motor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, position, velocity, voltage, supplyCurrent, statorCurrent, temp);
        motor.optimizeBusUtilization();

        motor.setPosition(MIN_ANGLE.getRotations());
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                        position, velocity, voltage, supplyCurrent, statorCurrent, temp)
                .isOK();
        inputs.position = Rotation2d.fromRotations(position.getValueAsDouble());
        inputs.velocityRPS = velocity.getValueAsDouble();
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageCtrlReq.withOutput(volts));
    }

    @Override
    public void setSetpoint(Rotation2d position) {
        motor.setControl(positionCtrlReq.withPosition(position.getRotations()));
    }

    @Override
    public void resetPosition(Rotation2d position) {
        motor.setPosition(position.getRotations());
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        motorConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void configPID(double kP, double kI, double kD, double kS, double kV, double kG) {
        motorConfig.Slot0.kP = kP;
        motorConfig.Slot0.kI = kI;
        motorConfig.Slot0.kD = kD;
        motorConfig.Slot0.kS = kS;
        motorConfig.Slot0.kV = kV;
        motorConfig.Slot0.kG = kG;
        motor.getConfigurator().apply(motorConfig);
    }
}
