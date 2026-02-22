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
    private final TalonFX spinMotor = new TalonFX(SPIN_MOTOR_ID);
    private final TalonFX feederMotor = new TalonFX(FEEDER_MOTOR_ID);

    private final StatusSignal<AngularVelocity> spinVelocity = spinMotor.getVelocity();
    private final StatusSignal<Voltage> spinVoltage = spinMotor.getMotorVoltage();
    private final StatusSignal<Current> spinSupplyCurrent = spinMotor.getSupplyCurrent();
    private final StatusSignal<Current> spinStatorCurrent = spinMotor.getStatorCurrent();
    private final StatusSignal<Temperature> spinTemp = spinMotor.getDeviceTemp();

    private final StatusSignal<AngularVelocity> feederVelocity = feederMotor.getVelocity();
    private final StatusSignal<Voltage> feederVoltage = feederMotor.getMotorVoltage();
    private final StatusSignal<Current> feederSupplyCurrent = feederMotor.getSupplyCurrent();
    private final StatusSignal<Current> feederStatorCurrent = feederMotor.getStatorCurrent();
    private final StatusSignal<Temperature> feederTemp = feederMotor.getDeviceTemp();

    private final VoltageOut spinVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);
    private final VoltageOut feederVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

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

        spinMotor.getConfigurator().apply(config);

        config.Feedback.SensorToMechanismRatio = FEEDER_GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederMotor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                spinVelocity,
                spinVoltage,
                spinSupplyCurrent,
                spinStatorCurrent,
                spinTemp,
                feederVelocity,
                feederVoltage,
                feederSupplyCurrent,
                feederStatorCurrent,
                feederTemp);

        spinMotor.optimizeBusUtilization();
        feederMotor.optimizeBusUtilization();
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
                        feederVelocity, feederVoltage, feederSupplyCurrent, feederStatorCurrent, feederTemp)
                .isOK();
        inputs.feederVelocityRPS = feederVelocity.getValueAsDouble();
        inputs.feederAppliedVolts = feederVoltage.getValueAsDouble();
        inputs.feederSupplyCurrentAmps = feederSupplyCurrent.getValueAsDouble();
        inputs.feederStatorCurrentAmps = feederStatorCurrent.getValueAsDouble();
        inputs.feederTempCelsius = feederTemp.getValueAsDouble();
    }

    @Override
    public void setMotorVoltages(double spinVolts, double feederVolts) {
        spinMotor.setControl(spinVoltageCtrlReq.withOutput(spinVolts));
        feederMotor.setControl(feederVoltageCtrlReq.withOutput(feederVolts));
    }
}
