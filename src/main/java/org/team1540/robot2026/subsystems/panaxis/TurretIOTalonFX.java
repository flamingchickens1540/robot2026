package org.team1540.robot2026.subsystems.panaxis;

import static edu.wpi.first.units.Units.Rotations;
import static org.team1540.robot2026.subsystems.panaxis.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;

public class TurretIOTalonFX implements TurretIO {
    // Magic Motion
    private final MotionMagicVoltage profiledPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

    // Drive Motor
    private final TalonFX driveMotor = new TalonFX(DRIVE_ID);
    private final StatusSignal<AngularVelocity> driveVelocity = driveMotor.getVelocity();
    private final StatusSignal<Angle> drivePosition = driveMotor.getPosition();
    private final StatusSignal<Voltage> driveAppliedVoltage = driveMotor.getMotorVoltage();
    private final StatusSignal<Current> driveMotorSupplyCurrent = driveMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> driveMotorTemp = driveMotor.getDeviceTemp();
    private final StatusSignal<Current> driveMotorStatorCurrent = driveMotor.getStatorCurrent();


    // Debouncer
    private final Debouncer leaderDebouncer = new Debouncer(0.5);

    public TurretIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        BaseStatusSignal.setUpdateFrequencyForAll(
                UPDATE_HRTZ,
                drivePosition,
                driveVelocity,
                driveAppliedVoltage,
                driveMotorSupplyCurrent,
                driveMotorTemp,
                driveMotorStatorCurrent);
    }

    public void updateInputs(TurretIO.TurretIOInputs inputs) {
        StatusCode driveStatus = BaseStatusSignal.refreshAll(
                drivePosition, driveVelocity,
                driveAppliedVoltage, driveMotorSupplyCurrent,
                driveMotorTemp, driveMotorStatorCurrent);

        inputs.driveConnected = leaderDebouncer.calculate(driveStatus.isOK());
        inputs.driveSupplyCurrentAmps = driveMotorSupplyCurrent.getValueAsDouble();
        inputs.driveAppliedVolts = driveAppliedVoltage.getValueAsDouble();
        inputs.driveTempCelsius = driveMotorTemp.getValueAsDouble();
        inputs.driveStatorCurrentAmps = driveMotorStatorCurrent.getValueAsDouble();
        inputs.drivePositionRads = drivePosition.getValueAsDouble();
        inputs.driveVelocityRadPerSec = driveVelocity.getValueAsDouble();
    }

    public void setVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    public void setRotation(double rotationDegrees) {
        driveMotor.setControl(profiledPositionControl.withPosition(
                Rotations.of(SUN_TO_DRIVE_RATIO * rotationDegrees)));
    }

    public void setBrakeMode(boolean brakeMode) {
        driveMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void configPID(double kP, double kI, double kD) {
        Slot0Configs configs = new Slot0Configs();
        driveMotor.getConfigurator().refresh(configs);
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        driveMotor.getConfigurator().apply(configs);
    }

    public void configFF(double kS, double kV, double kG) {
        Slot0Configs configs = new Slot0Configs();
        driveMotor.getConfigurator().refresh(configs);
        configs.kS = kS;
        configs.kV = kV;
        configs.kG = kG;
        driveMotor.getConfigurator().apply(configs);
    }
}
