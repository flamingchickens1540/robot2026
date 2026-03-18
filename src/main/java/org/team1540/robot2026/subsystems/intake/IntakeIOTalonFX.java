package org.team1540.robot2026.subsystems.intake;

import static org.team1540.robot2026.subsystems.intake.IntakeConstants.*;
import static org.team1540.robot2026.subsystems.intake.IntakeConstants.LEFT_INTAKE_MOTOR_ID;
import static org.team1540.robot2026.subsystems.intake.IntakeConstants.RIGHT_INTAKE_MOTOR_ID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX pivotMotor = new TalonFX(PIVOT_MOTOR_ID);
    private final TalonFX leftSpinMotor = new TalonFX(LEFT_INTAKE_MOTOR_ID);
    private final TalonFX rightSpinMotor = new TalonFX(RIGHT_INTAKE_MOTOR_ID);

    private final StatusSignal<AngularVelocity> rightSpinMotorVelocityRPS = rightSpinMotor.getVelocity();
    private final StatusSignal<Voltage> rightSpinMotorAppliedVolts = rightSpinMotor.getMotorVoltage();
    private final StatusSignal<Current> rightSpinSupplyCurrentAmps = rightSpinMotor.getSupplyCurrent();
    private final StatusSignal<Current> rightSpinStatorCurrentAmps = rightSpinMotor.getStatorCurrent();
    private final StatusSignal<Temperature> rightSpinMotorTemp = rightSpinMotor.getDeviceTemp();

    private final StatusSignal<AngularVelocity> leftSpinMotorVelocityRPS = leftSpinMotor.getVelocity();
    private final StatusSignal<Voltage> leftSpinMotorAppliedVolts = leftSpinMotor.getMotorVoltage();
    private final StatusSignal<Current> leftSpinSupplyCurrentAmps = leftSpinMotor.getSupplyCurrent();
    private final StatusSignal<Current> leftSpinStatorCurrentAmps = leftSpinMotor.getStatorCurrent();
    private final StatusSignal<Temperature> leftSpinMotorTemp = leftSpinMotor.getDeviceTemp();

    private final StatusSignal<Angle> pivotPositon = pivotMotor.getPosition();
    private final StatusSignal<AngularVelocity> pivotMotorVelocityRPS = pivotMotor.getVelocity();
    private final StatusSignal<Current> pivotStatorCurrentAmps = pivotMotor.getStatorCurrent();
    private final StatusSignal<Temperature> pivotMotorTemp = pivotMotor.getDeviceTemp();
    private final StatusSignal<Voltage> pivotMotorAppliedVolts = pivotMotor.getMotorVoltage();
    private final StatusSignal<Current> pivotSupplyCurrentAmps = pivotMotor.getSupplyCurrent();

    private final VoltageOut spinVoltageRequest = new VoltageOut(0);

    private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0).withSlot(0);

    public IntakeIOTalonFX() {

        TalonFXConfiguration pivotTalonFXConfig = new TalonFXConfiguration();
        pivotTalonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotTalonFXConfig.CurrentLimits.StatorCurrentLimit = 120;
        pivotTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 40;
        pivotTalonFXConfig.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive; // TODO: is it inverted or not?
        pivotTalonFXConfig.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;
        pivotTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration intakeTalonFXConfig = new TalonFXConfiguration();
        intakeTalonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeTalonFXConfig.CurrentLimits.StatorCurrentLimit = 120;
        intakeTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 70;
        intakeTalonFXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        intakeTalonFXConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        intakeTalonFXConfig.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive; // TODO: is it inverted or not?
        intakeTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

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

        pivotMotor.getConfigurator().apply(pivotTalonFXConfig);
        leftSpinMotor.getConfigurator().apply(intakeTalonFXConfig);
        rightSpinMotor.getConfigurator().apply(intakeTalonFXConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                leftSpinMotorVelocityRPS,
                leftSpinMotorAppliedVolts,
                leftSpinSupplyCurrentAmps,
                leftSpinStatorCurrentAmps,
                leftSpinMotorTemp,
                pivotMotorVelocityRPS,
                pivotPositon,
                pivotMotorAppliedVolts,
                pivotSupplyCurrentAmps,
                pivotStatorCurrentAmps,
                pivotMotorTemp);
        rightSpinMotor.optimizeBusUtilization();
        leftSpinMotor.optimizeBusUtilization();
        pivotMotor.optimizeBusUtilization();

        rightSpinMotor.setControl(new Follower(IntakeConstants.LEFT_INTAKE_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        StatusCode spinStatus = BaseStatusSignal.refreshAll(
                leftSpinMotorVelocityRPS,
                leftSpinMotorAppliedVolts,
                leftSpinSupplyCurrentAmps,
                leftSpinStatorCurrentAmps,
                leftSpinMotorTemp,
                rightSpinMotorVelocityRPS,
                rightSpinMotorAppliedVolts,
                rightSpinSupplyCurrentAmps,
                rightSpinStatorCurrentAmps,
                rightSpinMotorTemp);
        StatusCode pivotStatus = BaseStatusSignal.refreshAll(
                pivotMotorVelocityRPS,
                pivotPositon,
                pivotMotorAppliedVolts,
                pivotSupplyCurrentAmps,
                pivotStatorCurrentAmps,
                pivotMotorTemp);

        inputs.leftSpinConnected = spinStatus.isOK();
        inputs.leftSpinMotorVelocityRPS = leftSpinMotorVelocityRPS.getValueAsDouble();
        inputs.leftSpinMotorAppliedVolts = leftSpinMotorAppliedVolts.getValueAsDouble();
        inputs.leftSpinSupplyCurrentAmps = leftSpinSupplyCurrentAmps.getValueAsDouble();
        inputs.leftSpinStatorCurrentAmps = leftSpinStatorCurrentAmps.getValueAsDouble();
        inputs.leftSpinMotorTemp = leftSpinMotorTemp.getValueAsDouble();

        inputs.rightSpinConnected = spinStatus.isOK();
        inputs.rightSpinMotorVelocityRPS = rightSpinMotorVelocityRPS.getValueAsDouble();
        inputs.rightSpinMotorAppliedVolts = rightSpinMotorAppliedVolts.getValueAsDouble();
        inputs.rightSpinSupplyCurrentAmps = rightSpinSupplyCurrentAmps.getValueAsDouble();
        inputs.rightSpinStatorCurrentAmps = rightSpinStatorCurrentAmps.getValueAsDouble();
        inputs.rightSpinMotorTemp = rightSpinMotorTemp.getValueAsDouble();

        inputs.pivotConnected = pivotStatus.isOK();
        inputs.pivotPosition = Rotation2d.fromRotations(pivotPositon.getValueAsDouble());
        inputs.pivotMotorVelocityRPS = pivotMotorVelocityRPS.getValueAsDouble();
        inputs.pivotMotorAppliedVolts = pivotMotorAppliedVolts.getValueAsDouble();
        inputs.pivotSupplyCurrentAmps = pivotSupplyCurrentAmps.getValueAsDouble();
        inputs.pivotStatorCurrentAmps = pivotStatorCurrentAmps.getValueAsDouble();
        inputs.pivotMotorTemp = pivotMotorTemp.getValueAsDouble();
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        leftSpinMotor.setControl(spinVoltageRequest.withOutput(voltage));
    }

    @Override
    public void setPivotSetpoint(Rotation2d pivotPosition) {
        pivotMotor.setControl(pivotPositionRequest.withPosition(pivotPosition.getRotations()));
    }

    @Override
    public void resetPivotPosition(Rotation2d pivotPosition) {
        pivotMotor.setPosition(pivotPosition.getRotations());
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
