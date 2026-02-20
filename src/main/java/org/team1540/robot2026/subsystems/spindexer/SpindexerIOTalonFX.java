package org.team1540.robot2026.subsystems.spindexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import org.team1540.robot2026.Constants;

import static org.team1540.robot2026.subsystems.spindexer.SpindexerConstants.*;


public class SpindexerIOTalonFX implements SpindexerIO {
    private final TalonFX intakeMotor = new TalonFX(INTAKE_ID);
    private final TalonFX feederMotor = new TalonFX(FEEDER_ID);

    private final VoltageOut feederVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);
    private final VelocityVoltage feederVelocityCtrlReq = new VelocityVoltage(0).withEnableFOC(true);
    private final StatusSignal<Voltage> feederVoltage = feederMotor.getMotorVoltage();
    private final StatusSignal<Current> feederCurrent = feederMotor.getSupplyCurrent();
    private final StatusSignal<AngularVelocity> feederVelocity = feederMotor.getVelocity();
    private final StatusSignal<Temperature> feederTemp = feederMotor.getDeviceTemp();

    private final VoltageOut intakeVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);
    private final StatusSignal<Voltage> intakeVoltage = intakeMotor.getMotorVoltage();
    private final StatusSignal<Current> intakeCurrent = intakeMotor.getSupplyCurrent();
    private final StatusSignal<AngularVelocity> intakeVelocity = intakeMotor.getVelocity();
    private final StatusSignal<Temperature> intakeTemp = intakeMotor.getDeviceTemp();

    public SpindexerIOTalonFX() {
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 55;

        TalonFXConfiguration feederConfig  = new TalonFXConfiguration();
        feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederConfig.CurrentLimits.SupplyCurrentLimit = 60;
        feederConfig.Slot0.kP = FEEDER_KP;
        feederConfig.Slot0.kI = FEEDER_KI;
        feederConfig.Slot0.kD = FEEDER_KD;
        feederConfig.Slot0.kV = FEEDER_KV;

        intakeMotor.getConfigurator().apply(intakeConfig);
        feederMotor.getConfigurator().apply(feederConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(50,
                feederVoltage,
                feederCurrent,
                feederVelocity,
                feederTemp,
                intakeVoltage,
                intakeCurrent,
                intakeVelocity,
                intakeTemp);
        intakeMotor.optimizeBusUtilization();
        feederMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(SpindexerIO.SpindexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                intakeVoltage,
                intakeCurrent,
                intakeVelocity,
                intakeTemp,
                feederVoltage,
                feederCurrent,
                feederVelocity,
                feederTemp);
        inputs.intakeVoltage = intakeVoltage.getValueAsDouble();
        inputs.intakeCurrentAmps = intakeCurrent.getValueAsDouble();
        inputs.intakeVelocityRPM = intakeVelocity.getValueAsDouble() * 60;
        inputs.intakeTempCelsius = intakeTemp.getValueAsDouble();
        inputs.feederVoltage = feederVoltage.getValueAsDouble();
        inputs.feederCurrentAmps = feederCurrent.getValueAsDouble();
        inputs.feederVelocityRPM = feederVelocity.getValueAsDouble() * 60;
        inputs.feederTempCelsius = feederTemp.getValueAsDouble();
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotor.setControl(intakeVoltageCtrlReq.withOutput(volts));
    }

    @Override
    public void setFeederVoltage(double voltage) {
        feederMotor.setControl(feederVoltageCtrlReq.withOutput(voltage));
    }

    @Override
    public void setFeederVelocity(double velocityRPM) {
        feederMotor.setControl(feederVelocityCtrlReq.withVelocity(velocityRPM / 60));
    }

    @Override
    public void configureFeederPID(double p, double i, double d) {
        Slot0Configs pidConfigs = new Slot0Configs();
        feederMotor.getConfigurator().refresh(pidConfigs);
        pidConfigs.kP = p;
        pidConfigs.kI = i;
        pidConfigs.kD = d;
        feederMotor.getConfigurator().apply(pidConfigs);
    }

    @Override
    public void setIntakeBrakeMode(boolean isBrakeMode) {
        intakeMotor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

}
