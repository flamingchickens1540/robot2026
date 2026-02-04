package org.team1540.robot2026.subsystems.intake;

import static org.team1540.robot2026.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOTalonFX implements IntakeIO {

    private TalonFX pivotMotor = new TalonFX(PIVOT_MOTOR_ID);
    private TalonFX intakeMotor = new TalonFX(INTAKE_MOTOR_ID);

    public IntakeIOTalonFX() {

        TalonFXConfiguration pivotTalonFXConfig = new TalonFXConfiguration();
        pivotTalonFXConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
        pivotTalonFXConfig.CurrentLimits.withStatorCurrentLimit(120);
        pivotTalonFXConfig.CurrentLimits.withSupplyCurrentLimit(50);
        pivotTalonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: is it inverted
        pivotTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        pivotMotor.getConfigurator().apply(pivotTalonFXConfig);

        TalonFXConfiguration intakeTalonFXConfig = new TalonFXConfiguration();
        intakeTalonFXConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
        intakeTalonFXConfig.CurrentLimits.withStatorCurrentLimit(120);
        intakeTalonFXConfig.CurrentLimits.withSupplyCurrentLimit(50);
        intakeTalonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: is it inverted
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
    public void updateInputs(IntakeInputs inputs) {}
}
