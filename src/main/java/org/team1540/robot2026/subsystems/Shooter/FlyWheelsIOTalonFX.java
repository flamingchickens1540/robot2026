package org.team1540.robot2026.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static org.team1540.robot2026.subsystems.Shooter.ShooterConstants.Flywheels.*;

public class FlyWheelsIOTalonFX implements FlyWheelsIO{
    private final TalonFX motor = new TalonFX(ID);

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> supplyCurrent ;
    private final StatusSignal<Temperature> temperature ;




    private final VelocityVoltage velocityCtrlReq=
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VoltageOut voltageCtrlReq = new VoltageOut(0).withEnableFOC(true);


    public FlyWheelsIOTalonFX() {
        velocity = motor.getVelocity();
        appliedVoltage = motor.getMotorVoltage();
        statorCurrent = motor.getStatorCurrent();
        supplyCurrent = motor.getSupplyCurrent();
        temperature = motor.getDeviceTemp();

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = KP;
        config.Slot0.kI = KI;
        config.Slot0.kD = KD;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                velocity,
                appliedVoltage,
                statorCurrent,
                supplyCurrent,
                temperature);

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocity,
                appliedVoltage,
                statorCurrent,
                supplyCurrent,
                temperature
        );

        inputs.velocityRPM = velocity.getValueAsDouble();
        inputs.appliedVolts = appliedVoltage.getValueAsDouble();
        inputs.statorCurrentAmps=statorCurrent.getValueAsDouble();
        inputs.tempCelsius= temperature.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    }


    @Override
    public void setSpeeds(double RPM){
        motor.setControl(velocityCtrlReq.withVelocity(RPM/60));
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageCtrlReq.withOutput(volts));
    }
    @Override
    public void configPID(double kP, double kI, double kD, double kV, double kS){
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
