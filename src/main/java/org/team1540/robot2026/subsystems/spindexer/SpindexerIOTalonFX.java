package org.team1540.robot2026.subsystems.spindexer;

import static org.team1540.robot2026.subsystems.spindexer.SpindexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SpindexerIOTalonFX implements SpindexerIO {
    private final TalonFX spinMotor1 = new TalonFX(SPIN_MOTOR_ID);
    private final TalonFX feederMotor2 = new TalonFX(FEEDER_MOTOR_ID_2);
    private final TalonFX feederMotor1 = new TalonFX(FEEDER_MOTOR_ID_1);

    private final StatusSignal<AngularVelocity> spinVelocity = spinMotor1.getVelocity();
    private final StatusSignal<Voltage> spinVoltage = spinMotor1.getMotorVoltage();
    private final StatusSignal<Current> spinSupplyCurrent = spinMotor1.getSupplyCurrent();
    private final StatusSignal<Current> spinStatorCurrent = spinMotor1.getStatorCurrent();
    private final StatusSignal<Temperature> spinTemp = spinMotor1.getDeviceTemp();

    private final StatusSignal<AngularVelocity> feeder1Velocity = feederMotor1.getVelocity();
    private final StatusSignal<Voltage> feeder1Voltage = feederMotor1.getMotorVoltage();
    private final StatusSignal<Current> feeder1SupplyCurrent = feederMotor1.getSupplyCurrent();
    private final StatusSignal<Current> feeder1StatorCurrent = feederMotor1.getStatorCurrent();
    private final StatusSignal<Temperature> feeder1Temp = feederMotor1.getDeviceTemp();

    private final StatusSignal<AngularVelocity> feeder2Velocity = feederMotor2.getVelocity();
    private final StatusSignal<Voltage> feeder2Voltage = feederMotor2.getMotorVoltage();
    private final StatusSignal<Current> feeder2SupplyCurrent = feederMotor2.getSupplyCurrent();
    private final StatusSignal<Current> feeder2StatorCurrent = feederMotor2.getStatorCurrent();
    private final StatusSignal<Temperature> feeder2Temp = feederMotor2.getDeviceTemp();

    private final VoltageOut spinVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);
    private final VoltageOut feederVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);
    private final VoltageOut feeder2VoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

    public SpindexerIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = SPIN_GEAR_RATIO;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80.0;

        spinMotor1.getConfigurator().apply(config);

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = FEEDER_GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        feederMotor1.getConfigurator().apply(config);
        feederMotor2.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                spinVelocity,
                spinVoltage,
                spinSupplyCurrent,
                spinStatorCurrent,
                spinTemp,
                feeder1Velocity,
                feeder1Voltage,
                feeder1SupplyCurrent,
                feeder1StatorCurrent,
                feeder1Temp,
                feeder2Velocity,
                feeder2Voltage,
                feeder2SupplyCurrent,
                feeder2StatorCurrent,
                feeder2Temp);

        spinMotor1.optimizeBusUtilization();
        feederMotor1.optimizeBusUtilization();
        feederMotor2.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.spinMotorConnected = BaseStatusSignal.refreshAll(
                        spinVelocity, spinVoltage, spinSupplyCurrent, spinStatorCurrent, spinTemp)
                .isOK();
        inputs.spinVelocityRPS = spinVelocity.getValueAsDouble();
        inputs.spinAppliedVolts = spinVoltage.getValueAsDouble();
        inputs.spinSupplyCurrentAmps = spinSupplyCurrent.getValueAsDouble();
        inputs.spinStatorCurrentAmps = spinStatorCurrent.getValueAsDouble();
        inputs.spinTempCelsius = spinTemp.getValueAsDouble();

        inputs.feederMotorConnected = BaseStatusSignal.refreshAll(
                        feeder1Velocity,
                        feeder1Voltage,
                        feeder1SupplyCurrent,
                        feeder1StatorCurrent,
                        feeder2Temp,
                        feeder2Velocity,
                        feeder2Voltage,
                        feeder2SupplyCurrent,
                        feeder2StatorCurrent,
                        feeder2Temp)
                .isOK();

        inputs.feeder2MotorConnected = BaseStatusSignal.refreshAll(
                        feeder2Temp,
                        feeder2Velocity,
                        feeder2Voltage,
                        feeder2SupplyCurrent,
                        feeder2StatorCurrent,
                        feeder2Temp)
                .isOK();
        inputs.feeder1VelocityRPS = feeder1Velocity.getValueAsDouble();
        inputs.feeder1AppliedVolts = feeder1Voltage.getValueAsDouble();
        inputs.feeder1SupplyCurrentAmps = feeder1SupplyCurrent.getValueAsDouble();
        inputs.feeder1StatorCurrentAmps = feeder1StatorCurrent.getValueAsDouble();
        inputs.feeder1TempCelsius = feeder1Temp.getValueAsDouble();

        inputs.feeder2VelocityRPS = feeder2Velocity.getValueAsDouble();
        inputs.feeder2AppliedVolts = feeder2Voltage.getValueAsDouble();
        inputs.feeder2SupplyCurrentAmps = feeder2SupplyCurrent.getValueAsDouble();
        inputs.feeder2StatorCurrentAmps = feeder2StatorCurrent.getValueAsDouble();
        inputs.feeder2TempCelsius = feeder2Temp.getValueAsDouble();
    }

    @Override
    public void setMotorVoltages(double spinVolts, double feeder1Volts, double feeder2Volts) {
        spinMotor1.setControl(spinVoltageCtrlReq.withOutput(spinVolts));
        feederMotor1.setControl(feederVoltageCtrlReq.withOutput(feeder1Volts));
        feederMotor2.setControl(feederVoltageCtrlReq.withOutput(feeder2Volts));
    }
}
