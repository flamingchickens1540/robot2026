package org.team1540.robot2026.subsystems.intake;

import static org.team1540.robot2026.subsystems.intake.IntakeConstants.*;

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
import org.team1540.robot2026.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX pivotMotor = new TalonFX(PIVOT_MOTOR_ID);
    private final TalonFX rightSpinMotor = new TalonFX(RIGHT_SPIN_MOTOR_ID);
    private final TalonFX leftSpinMotor = new TalonFX(LEFT_SPIN_MOTOR_ID);

    private final StatusSignal<AngularVelocity> leftSpinVelocity = leftSpinMotor.getVelocity();
    private final StatusSignal<Voltage> leftSpinVoltage = leftSpinMotor.getMotorVoltage();
    private final StatusSignal<Current> leftSpinSupplyCurrent = leftSpinMotor.getSupplyCurrent();
    private final StatusSignal<Current> leftSpinStatorCurrent = leftSpinMotor.getStatorCurrent();
    private final StatusSignal<Temperature> leftSpinMotorTemp = leftSpinMotor.getDeviceTemp();

    private final StatusSignal<AngularVelocity> rightSpinVelocity = rightSpinMotor.getVelocity();
    private final StatusSignal<Voltage> rightSpinVoltage = rightSpinMotor.getMotorVoltage();
    private final StatusSignal<Current> rightSpinSupplyCurrent = rightSpinMotor.getSupplyCurrent();
    private final StatusSignal<Current> rightSpinStatorCurrent = rightSpinMotor.getStatorCurrent();
    private final StatusSignal<Temperature> rightSpinMotorTemp = rightSpinMotor.getDeviceTemp();

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

        TalonFXConfiguration intakeTalonFXConfig = new TalonFXConfiguration();
        intakeTalonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeTalonFXConfig.CurrentLimits.StatorCurrentLimit = 120;
        intakeTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 80;
        intakeTalonFXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        intakeTalonFXConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        intakeTalonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeTalonFXConfig.Feedback.SensorToMechanismRatio = SPIN_GEAR_RATIO;
        intakeTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        PhoenixUtil.tryUntilOk(5, () -> leftSpinMotor.getConfigurator().apply(intakeTalonFXConfig));
        PhoenixUtil.tryUntilOk(5, () -> rightSpinMotor.getConfigurator().apply(intakeTalonFXConfig));
        leftSpinMotor.setControl(new Follower(rightSpinMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                leftSpinVelocity,
                leftSpinVoltage,
                leftSpinSupplyCurrent,
                leftSpinStatorCurrent,
                leftSpinMotorTemp,
                rightSpinVelocity,
                rightSpinVoltage,
                rightSpinSupplyCurrent,
                rightSpinStatorCurrent,
                rightSpinMotorTemp,
                pivotMotorVelocityRPS,
                pivotPositon,
                pivotMotorAppliedVolts,
                pivotSupplyCurrentAmps,
                pivotStatorCurrentAmps,
                pivotMotorTemp);
        rightSpinMotor.optimizeBusUtilization();
        pivotMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.rightSpinConnected = BaseStatusSignal.refreshAll(
                        rightSpinVelocity,
                        rightSpinVoltage,
                        rightSpinSupplyCurrent,
                        rightSpinStatorCurrent,
                        rightSpinMotorTemp)
                .isOK();
        inputs.leftSpinConnected = BaseStatusSignal.refreshAll(
                        leftSpinVelocity,
                        leftSpinVoltage,
                        leftSpinSupplyCurrent,
                        leftSpinStatorCurrent,
                        leftSpinMotorTemp)
                .isOK();
        inputs.spinMotorVelocityRPS =
                new double[] {leftSpinVelocity.getValueAsDouble(), rightSpinVelocity.getValueAsDouble()};
        inputs.spinMotorAppliedVolts =
                new double[] {leftSpinVoltage.getValueAsDouble(), rightSpinVoltage.getValueAsDouble()};
        inputs.spinSupplyCurrentAmps =
                new double[] {leftSpinSupplyCurrent.getValueAsDouble(), rightSpinSupplyCurrent.getValueAsDouble()};
        inputs.spinStatorCurrentAmps =
                new double[] {leftSpinStatorCurrent.getValueAsDouble(), rightSpinStatorCurrent.getValueAsDouble()};
        inputs.spinMotorTemp =
                new double[] {leftSpinMotorTemp.getValueAsDouble(), rightSpinMotorTemp.getValueAsDouble()};

        StatusCode pivotStatus = BaseStatusSignal.refreshAll(
                pivotMotorVelocityRPS,
                pivotPositon,
                pivotMotorAppliedVolts,
                pivotSupplyCurrentAmps,
                pivotStatorCurrentAmps,
                pivotMotorTemp);
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
        rightSpinMotor.setControl(spinVoltageRequest.withOutput(voltage));
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
        PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(configs));
    }

    @Override
    public void setPivotFF(double kS, double kV, double kG) {
        Slot0Configs configs = new Slot0Configs();
        pivotMotor.getConfigurator().refresh(configs);
        configs.kS = kS;
        configs.kV = kV;
        configs.kA = kG;
        PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(configs));
    }
}
