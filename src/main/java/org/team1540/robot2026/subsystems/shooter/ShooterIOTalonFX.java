package org.team1540.robot2026.subsystems.shooter;

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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX rightMotor = new TalonFX(ShooterConstants.RIGHT_ID);
    private final TalonFX leftMotor = new TalonFX(ShooterConstants.LEFT_ID);

    private final StatusSignal<AngularVelocity> leftVelocity = rightMotor.getVelocity();
    private final StatusSignal<Voltage> leftAppliedVoltage = rightMotor.getMotorVoltage();
    private final StatusSignal<Current> leftStatorCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Current> leftSupplyCurrent = rightMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> leftTemperature = rightMotor.getDeviceTemp();

    private final StatusSignal<AngularVelocity> rightVelocity = rightMotor.getVelocity();
    private final StatusSignal<Voltage> rightAppliedVoltage = rightMotor.getMotorVoltage();
    private final StatusSignal<Current> rightStatorCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Current> rightSupplyCurrent = rightMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> rightTemperature = rightMotor.getDeviceTemp();

    private final VelocityVoltage velocityCtrlReq =
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VoltageOut voltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

    public ShooterIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = ShooterConstants.KP;
        config.Slot0.kI = ShooterConstants.KI;
        config.Slot0.kD = ShooterConstants.KD;
        config.Slot0.kS = ShooterConstants.KS;
        config.Slot0.kV = ShooterConstants.KV;

        config.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightMotor.getConfigurator().apply(config);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotor.getConfigurator().apply(config);

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

        leftMotor.setControl(new Follower(ShooterConstants.RIGHT_ID, MotorAlignmentValue.Opposed));

        rightMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftMotorConnected = BaseStatusSignal.refreshAll(
                        leftVelocity, leftAppliedVoltage, leftStatorCurrent, leftSupplyCurrent, leftTemperature)
                .isOK();
        inputs.leftVelocityRPM = leftVelocity.getValueAsDouble();
        inputs.leftAppliedVolts = leftAppliedVoltage.getValueAsDouble();
        inputs.leftStatorCurrentAmps = leftStatorCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTemperature.getValueAsDouble();
        inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();

        inputs.rightMotorConnected = BaseStatusSignal.refreshAll(
                        rightVelocity,
                        rightAppliedVoltage,
                        rightAppliedVoltage,
                        rightStatorCurrent,
                        rightSupplyCurrent,
                        rightTemperature)
                .isOK();
        inputs.rightVelocityRPM = rightVelocity.getValueAsDouble();
        inputs.rightAppliedVolts = rightAppliedVoltage.getValueAsDouble();
        inputs.rightStatorCurrentAmps = rightStatorCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTemperature.getValueAsDouble();
        inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    }

    @Override
    public void setSpeed(double rpm) {
        rightMotor.setControl(velocityCtrlReq.withVelocity(rpm / 60));
    }

    @Override
    public void setVoltage(double volts) {
        rightMotor.setControl(voltageCtrlReq.withOutput(volts));
    }

    @Override
    public void configPID(double kP, double kI, double kD, double kS, double kV) {
        Slot0Configs pidConfigs = new Slot0Configs();
        rightMotor.getConfigurator().refresh(pidConfigs);
        pidConfigs.kP = kP;
        pidConfigs.kI = kI;
        pidConfigs.kD = kD;
        pidConfigs.kS = kS;
        pidConfigs.kV = kV;
        rightMotor.getConfigurator().apply(pidConfigs);
    }
}
