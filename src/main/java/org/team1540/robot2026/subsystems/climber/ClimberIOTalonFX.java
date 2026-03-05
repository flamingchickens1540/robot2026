package org.team1540.robot2026.subsystems.climber;

import static org.team1540.robot2026.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX motor = new TalonFX(MOTOR_ID);
    private final StatusSignal<Angle> position = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
    private final StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent();
    private final StatusSignal<Temperature> temp = motor.getDeviceTemp();

    private final MotionMagicVoltage profiledPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

    public ClimberIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        Slot0Configs gains = config.Slot0;
        gains.kS = KS;
        gains.kV = KV;
        gains.kG = KG;
        gains.kP = KP;
        gains.kI = KI;
        gains.kD = KD;
        gains.GravityType = GravityTypeValue.Elevator_Static;

        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = CRUISE_VELOCITY_MPS;
        motionMagicConfigs.MotionMagicAcceleration = ACCELERATION_MPS2;

        motor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(50, position, velocity, voltage, supplyCurrent, statorCurrent, temp);

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                        position, velocity, voltage, supplyCurrent, statorCurrent, temp)
                .isOK();
        inputs.positionMeters = position.getValueAsDouble();
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.velocityMPS = 2 * Math.PI * SPROCKET_RADIUS_M * velocity.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setSetpoint(double setpointMeters) {
        motor.setControl(profiledPositionControl.withPosition(setpointMeters));
    }

    @Override
    public void setBrakeMode(boolean setBrake) {
        motor.setNeutralMode(setBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void configFF(double kS, double kV, double kG) {
        Slot0Configs configs = new Slot0Configs();
        motor.getConfigurator().refresh(configs);
        configs.kS = kS;
        configs.kV = kV;
        configs.kG = kG;
        motor.getConfigurator().apply(configs);
    }

    @Override
    public void configPID(double kP, double kI, double kD) {
        Slot0Configs configs = new Slot0Configs();
        motor.getConfigurator().refresh(configs);
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        motor.getConfigurator().apply(configs);
    }
}
