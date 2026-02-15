package org.team1540.robot2026.subsystems.intake;

import static org.team1540.robot2026.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {

    private TalonFX pivotMotor = new TalonFX(PIVOT_MOTOR_ID);
    private TalonFX intakeMotor = new TalonFX(INTAKE_MOTOR_ID);

    private StatusSignal<AngularVelocity> spinMotorVelocityRPS;
    private StatusSignal<Voltage> spinMotorAppliedVolts;
    private StatusSignal<Current> spinSupplyCurrentAmps;
    private StatusSignal<Current> spinStatorCurrentAmps;

    private StatusSignal<Angle> pivotSetpoint;
    private StatusSignal<AngularVelocity> pivotMotorVelocityRPS;
    private StatusSignal<Voltage> pivotMotorAppliedVolts;
    private StatusSignal<Current> pivotSupplyCurrentAmps;
    private StatusSignal<Current> pivotStatorCurrentAmps;

    public IntakeIOTalonFX() {

        spinMotorVelocityRPS = intakeMotor.getVelocity();
        spinMotorAppliedVolts = intakeMotor.getMotorVoltage();
        spinSupplyCurrentAmps = intakeMotor.getSupplyCurrent();
        spinStatorCurrentAmps = intakeMotor.getStatorCurrent();

        pivotSetpoint = pivotMotor.getPosition();
        pivotMotorVelocityRPS = pivotMotor.getVelocity();
        pivotMotorAppliedVolts = pivotMotor.getMotorVoltage();
        pivotSupplyCurrentAmps = pivotMotor.getSupplyCurrent();
        pivotStatorCurrentAmps = pivotMotor.getStatorCurrent();

        TalonFXConfiguration pivotTalonFXConfig = new TalonFXConfiguration();
        pivotTalonFXConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
        pivotTalonFXConfig.CurrentLimits.withStatorCurrentLimit(120);
        pivotTalonFXConfig.CurrentLimits.withSupplyCurrentLimit(50);
        pivotTalonFXConfig.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive; // TODO: is it inverted or not?
        pivotTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        pivotMotor.getConfigurator().apply(pivotTalonFXConfig);

        TalonFXConfiguration intakeTalonFXConfig = new TalonFXConfiguration();
        intakeTalonFXConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
        intakeTalonFXConfig.CurrentLimits.withStatorCurrentLimit(120);
        intakeTalonFXConfig.CurrentLimits.withSupplyCurrentLimit(50);
        intakeTalonFXConfig.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive; // TODO: is it inverted or not?
        intakeTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        intakeMotor.getConfigurator().apply(pivotTalonFXConfig);

        Slot0Configs pivotGains = pivotTalonFXConfig.Slot0;
        pivotGains.kS = PIVOT_KS;
        pivotGains.kV = PIVOT_KV;
        pivotGains.kG = PIVOT_KG;
        pivotGains.kP = PIVOT_KP;
        pivotGains.kI = PIVOT_KI;
        pivotGains.kD = PIVOT_KD;

        MotionMagicConfigs motionMagicConfig = pivotTalonFXConfig.MotionMagic;
        motionMagicConfig.MotionMagicCruiseVelocity = PIVOT_CRUISE_VELOCITY_RPS;
        motionMagicConfig.MotionMagicAcceleration = PIVOT_ACCELERATION_RPS2;
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        StatusCode spinStatus = BaseStatusSignal.refreshAll(
                spinMotorVelocityRPS, spinMotorAppliedVolts, spinSupplyCurrentAmps, spinStatorCurrentAmps);
        StatusCode pivotStatus = BaseStatusSignal.refreshAll(
                pivotMotorVelocityRPS,
                pivotSetpoint,
                pivotMotorAppliedVolts,
                pivotSupplyCurrentAmps,
                pivotStatorCurrentAmps);

        // inputs.spinConnected = spinConnectedDebounce.calculate(spinStatus.isOK()); remake this to work for this

        inputs.spinMotorVelocityRPS = spinMotorVelocityRPS.getValueAsDouble();
        inputs.spinMotorAppliedVolts = spinMotorAppliedVolts.getValueAsDouble();
        inputs.spinSupplyCurrentAmps = spinSupplyCurrentAmps.getValueAsDouble();
        inputs.spinStatorCurrentAmps = spinStatorCurrentAmps.getValueAsDouble();

        // inputs.pivotConnected = pivotConnectedDebounce.calculate(pivotStatus.isOK()); remake that to work for this

        inputs.pivotPosition = Rotation2d.fromRotations(pivotSetpoint.getValueAsDouble());
        inputs.pivotMotorVelocityRPS = pivotMotorVelocityRPS.getValueAsDouble();
        inputs.pivotMotorAppliedVolts = pivotMotorAppliedVolts.getValueAsDouble();
        inputs.pivotSupplyCurrentAmps = pivotSupplyCurrentAmps.getValueAsDouble();
        inputs.pivotStatorCurrentAmps = pivotStatorCurrentAmps.getValueAsDouble();
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setPivotSetpoint(Rotation2d pivotPosition) {
        pivotMotor.setControl((ControlRequest) pivotPosition);
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setPivotPID(double kP, double kI, double kD) {
        Slot0Configs configs = new Slot0Configs();
        pivotMotor.getConfigurator().refresh(configs);
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        pivotMotor.getConfigurator().apply(configs);
    }

    @Override
    public void setPivotFF(double kS, double kV, double kG) {
        Slot0Configs configs = new Slot0Configs();
        pivotMotor.getConfigurator().refresh(configs);
        configs.kS = kS;
        configs.kV = kV;
        configs.kA = kG;
        pivotMotor.getConfigurator().apply(configs);
    }
}
