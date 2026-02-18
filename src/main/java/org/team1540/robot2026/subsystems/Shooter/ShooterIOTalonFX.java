package org.team1540.robot2026.subsystems.Shooter;

import static org.team1540.robot2026.subsystems.Shooter.ShooterConstants.Flywheels.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements FlywheelsIO {
    private final TalonFX motor = new TalonFX(ID);
    private final TalonFX motor2 = new TalonFX(ID2);

    private final StatusSignal<AngularVelocity> leftVelocity = motor.getVelocity();
    private final StatusSignal<Voltage> leftAppliedVoltage = motor.getMotorVoltage();
    private final StatusSignal<Current> leftStatorCurrent = motor.getStatorCurrent();
    private final StatusSignal<Current> leftSupplyCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> leftTemperature = motor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> rightVelocity = motor.getVelocity();
    private final StatusSignal<Voltage> rightAppliedVoltage = motor.getMotorVoltage();
    private final StatusSignal<Current> rightStatorCurrent = motor.getStatorCurrent();
    private final StatusSignal<Current> rightSupplyCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> rightTemperature = motor.getDeviceTemp();

    private final VelocityVoltage velocityCtrlReq =
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VoltageOut voltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

    public ShooterIOTalonFX() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = KP;
        config.Slot0.kI = KI;
        config.Slot0.kD = KD;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(config);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor2.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                leftVelocity,
                leftAppliedVoltage,
                leftStatorCurrent,
                leftSupplyCurrent,
                leftTemperature,
                rightVelocity,
                rightAppliedVoltage,
                rightStatorCurrent,
                rightSupplyCurrent,
                rightTemperature);

        motor2.setControl(new Follower(ID, MotorAlignmentValue.Opposed));

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        BaseStatusSignal.refreshAll(leftVelocity,
                leftAppliedVoltage,
                leftStatorCurrent,
                leftSupplyCurrent,
                leftTemperature,
                rightVelocity,
                rightAppliedVoltage,
                rightAppliedVoltage,
                rightStatorCurrent,
                rightSupplyCurrent,
                rightTemperature);

        inputs.leftVelocityRPM = leftVelocity.getValueAsDouble();
        inputs.leftAppliedVolts = leftAppliedVoltage.getValueAsDouble();
        inputs.leftStatorCurrentAmps = leftStatorCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTemperature.getValueAsDouble();
        inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
        inputs.rightVelocityRPM = rightVelocity.getValueAsDouble();
        inputs.rightAppliedVolts = rightAppliedVoltage.getValueAsDouble();
        inputs.rightStatorCurrentAmps = rightStatorCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTemperature.getValueAsDouble();
        inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    }

    @Override
    public void setSpeeds(double leftRPM, double rightRPM) {
        motor.setControl(velocityCtrlReq.withVelocity(leftRPM / 60));
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        motor.setControl(voltageCtrlReq.withOutput(leftVolts));
    }

    @Override
    public void configPID(double kP, double kI, double kD, double kV, double kS) {
        Slot0Configs pidConfigs = new Slot0Configs();
        motor.getConfigurator().refresh(pidConfigs);
        pidConfigs.kP = kP;
        pidConfigs.kI = kI;
        pidConfigs.kD = kD;
        pidConfigs.kV = kV;
        pidConfigs.kS = kS;
        motor.getConfigurator().apply(pidConfigs);
    }
}
