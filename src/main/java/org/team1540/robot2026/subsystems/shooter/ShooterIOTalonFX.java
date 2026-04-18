package org.team1540.robot2026.subsystems.shooter;

import static org.team1540.robot2026.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
import org.team1540.robot2026.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX rightMotor = new TalonFX(RIGHT_ID);
    private final TalonFX leftMotor = new TalonFX(LEFT_ID);

    private final StatusSignal<AngularVelocity> leftVelocity = leftMotor.getVelocity();
    private final StatusSignal<Voltage> leftAppliedVoltage = leftMotor.getMotorVoltage();
    private final StatusSignal<Current> leftTorqueCurrent = leftMotor.getTorqueCurrent();
    private final StatusSignal<Current> leftStatorCurrent = leftMotor.getStatorCurrent();
    private final StatusSignal<Current> leftSupplyCurrent = leftMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> leftTemperature = leftMotor.getDeviceTemp();

    private final StatusSignal<AngularVelocity> rightVelocity = rightMotor.getVelocity();
    private final StatusSignal<Voltage> rightAppliedVoltage = rightMotor.getMotorVoltage();
    private final StatusSignal<Current> rightTorqueCurrent = rightMotor.getTorqueCurrent();
    private final StatusSignal<Current> rightStatorCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Current> rightSupplyCurrent = rightMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> rightTemperature = rightMotor.getDeviceTemp();

    private final VelocityVoltage velocityVoltageCtrlReq =
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0).withUpdateFreqHz(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCtrlReq = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final VoltageOut voltageCtrlReq =
            new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(0);

    public ShooterIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        if (TORQUE_CONTROL) {
            config.Slot0.kP = TORQUE_KP;
            config.Slot0.kI = TORQUE_KI;
            config.Slot0.kD = TORQUE_KD;
            config.Slot0.kS = TORQUE_KS;
            config.Slot0.kV = TORQUE_KV;
        } else {
            config.Slot0.kP = VOLTAGE_KP;
            config.Slot0.kI = VOLTAGE_KI;
            config.Slot0.kD = VOLTAGE_KD;
            config.Slot0.kS = VOLTAGE_KS;
            config.Slot0.kV = VOLTAGE_KV;
        }

        config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config));

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));

        BaseStatusSignal.setUpdateFrequencyForAll(250, rightAppliedVoltage, rightTorqueCurrent);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                leftVelocity,
                leftAppliedVoltage,
                leftTorqueCurrent,
                leftStatorCurrent,
                leftSupplyCurrent,
                leftTemperature,
                rightVelocity,
                rightStatorCurrent,
                rightSupplyCurrent,
                rightTemperature);

        rightMotor.optimizeBusUtilization();
        leftMotor.optimizeBusUtilization();

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftMotorConnected = BaseStatusSignal.refreshAll(
                        leftVelocity,
                        leftAppliedVoltage,
                        leftTorqueCurrent,
                        leftStatorCurrent,
                        leftSupplyCurrent,
                        leftTemperature)
                .isOK();
        inputs.leftVelocityRPM = leftVelocity.getValueAsDouble() * 60;
        inputs.leftAppliedVolts = leftAppliedVoltage.getValueAsDouble();
        inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValueAsDouble();
        inputs.leftStatorCurrentAmps = leftStatorCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTemperature.getValueAsDouble();
        inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();

        inputs.rightMotorConnected = BaseStatusSignal.refreshAll(
                        rightVelocity,
                        rightAppliedVoltage,
                        rightTorqueCurrent,
                        rightStatorCurrent,
                        rightSupplyCurrent,
                        rightTemperature)
                .isOK();
        inputs.rightVelocityRPM = rightVelocity.getValueAsDouble() * 60;
        inputs.rightAppliedVolts = rightAppliedVoltage.getValueAsDouble();
        inputs.rightTorqueCurrentAmps = rightTorqueCurrent.getValueAsDouble();
        inputs.rightStatorCurrentAmps = rightStatorCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTemperature.getValueAsDouble();
        inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    }

    @Override
    public void runVelocityVoltage(double rpm) {
        rightMotor.setControl(velocityVoltageCtrlReq.withVelocity(rpm / 60));
    }

    @Override
    public void runVelocityTorque(double rpm) {
        rightMotor.setControl(velocityTorqueCtrlReq.withVelocity(rpm / 60));
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
        PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(pidConfigs));
        PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(pidConfigs));
    }
}
