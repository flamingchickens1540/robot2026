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
import edu.wpi.first.wpilibj.DigitalInput;
public class ClimberIOReal implements ClimberIO {
    private final TalonFX leftMotor = new TalonFX(LEFT_MOTOR_ID);
    private final StatusSignal<Angle> leftPosition = leftMotor.getPosition();
    private final StatusSignal<AngularVelocity> leftVelocity = leftMotor.getVelocity();
    private final StatusSignal<Voltage> leftVolts = leftMotor.getMotorVoltage();
    private final StatusSignal<Current> leftSupplyCurrent = leftMotor.getSupplyCurrent();
    private final StatusSignal<Current> leftSatorCurrent = leftMotor.getStatorCurrent();
    private final StatusSignal<Temperature> leftTempC = leftMotor.getDeviceTemp();

    private final TalonFX rightMotor = new TalonFX(RIGHT_MOTOR_ID);
    private final StatusSignal<Angle> rightPosition = rightMotor.getPosition();
    private final StatusSignal<AngularVelocity> rightVelocity = rightMotor.getVelocity();
    private final StatusSignal<Voltage> rightVolts = rightMotor.getMotorVoltage();
    private final StatusSignal<Current> rightSupplyCurrent = rightMotor.getSupplyCurrent();
    private final StatusSignal<Current> rightSatorCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Temperature> rightTempC = rightMotor.getDeviceTemp();
    private final MotionMagicVoltage profiledPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

    private final DigitalInput upperLimitSwitch = new DigitalInput(UPPER_LIMIT_ID);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_ID);

    public ClimberIOReal() {
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;


        Slot0Configs Gains = leftConfig.Slot0;
        Gains.kS = KS;
        Gains.kV = KV;
        Gains.kG = KG;
        Gains.kP = KP;
        Gains.kI = KI;
        Gains.kD = KD;
        Gains.GravityType = GravityTypeValue.Elevator_Static;


        MotionMagicConfigs leftMotionMagicConfigs = leftConfig.MotionMagic;
        MotionMagicConfigs rightMotionMagicConfigs = rightConfig.MotionMagic;

        leftMotionMagicConfigs.MotionMagicCruiseVelocity = CRUISE_VELOCITY_MPS;
        leftMotionMagicConfigs.MotionMagicAcceleration = ACCELERATION_MPS2;

        rightMotionMagicConfigs.MotionMagicCruiseVelocity = CRUISE_VELOCITY_MPS;
        rightMotionMagicConfigs.MotionMagicAcceleration = ACCELERATION_MPS2;

        leftMotor.getConfigurator().apply(leftConfig);
        rightMotor.getConfigurator().apply(rightConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                leftVelocity,
                leftVolts,
                leftSupplyCurrent,
                leftSatorCurrent,
                leftTempC,
                rightVelocity,
                rightVolts,
                rightSatorCurrent,
                rightSupplyCurrent,
                rightTempC);

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftVelocity,
                leftVolts,
                leftSupplyCurrent,
                leftSatorCurrent,
                leftTempC,
                rightVelocity,
                rightVolts,
                rightSatorCurrent,
                rightSupplyCurrent,
                rightTempC);

        inputs.leftMotorConnected = leftMotor.isConnected();
        inputs.leftMotorPosition = leftPosition.getValueAsDouble();
        inputs.leftMotorAppliedVolts = leftVolts.getValueAsDouble();
        inputs.leftMotorVelocityRPM = leftVelocity.getValueAsDouble();
        inputs.leftMotorStatorCurrentAmps = leftSatorCurrent.getValueAsDouble();
        inputs.leftMotorSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
        inputs.leftMotorTempC = leftTempC.getValueAsDouble();

        inputs.rightMotorConnected = rightMotor.isConnected();
        inputs.rightMotorPosition = rightPosition.getValueAsDouble();
        inputs.rightMotorAppliedVolts = rightVolts.getValueAsDouble();
        inputs.rightMotorVelocityRPM = rightVelocity.getValueAsDouble();
        inputs.rightMotorStatorCurrentAmps = rightSatorCurrent.getValueAsDouble();
        inputs.rightMotorSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
        inputs.rightMotorTempC = rightTempC.getValueAsDouble();

        inputs.atUpperLimit = !upperLimitSwitch.get();
        inputs.atLowerLimit = !lowerLimitSwitch.get();
    }

    @Override
    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void setSetPoint(double setpointMeters) {
        leftMotor.setControl(profiledPositionControl.withPosition(setpointMeters));
        rightMotor.setControl(profiledPositionControl.withPosition(setpointMeters));
    }

    @Override
    public void setBrakeMode(boolean setBrake) {
        leftMotor.setNeutralMode(setBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        rightMotor.setNeutralMode(setBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void configFF(double kS, double kV, double kG) {

    }

    @Override
    public void configPID(double kP, double kI, double kD) {
        Slot0Configs configs = new Slot0Configs();
        leftMotor.getConfigurator().refresh(configs);
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        leftMotor.getConfigurator().apply(configs);
        rightMotor.getConfigurator().apply(configs);
    }
}
