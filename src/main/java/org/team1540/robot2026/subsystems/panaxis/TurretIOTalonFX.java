package org.team1540.robot2026.subsystems.panaxis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.CAN;
import jdk.jshell.Snippet;

import static org.team1540.robot2026.subsystems.panaxis.TurretConstants.*;

public class TurretIOTalonFX implements TurretIO{
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

    public TurretIOTalonFX () {
        //TalonFXConfigurator config = new TalonFXConfigurator(); TODO Config Stuff

        BaseStatusSignal.setUpdateFrequencyForAll(
                UPDATE_HRTZ,
                drivePosition, driveVelocity,
                driveAppliedVoltage, driveMotorSupplyCurrent,
                driveMotorTemp, driveMotorStatorCurrent
        );
    }

    public void updateInputs (TurretIO.TurretIOInputs inputs) {
        StatusCode driveStatus = BaseStatusSignal.refreshAll(
                drivePosition, driveVelocity,
                driveAppliedVoltage, driveMotorSupplyCurrent,
                driveMotorTemp, driveMotorStatorCurrent
        );

        inputs.driveConnected = leaderDebouncer.calculate(driveStatus.isOK());
        inputs.driveSupplyCurrentAmps = driveMotorSupplyCurrent.getValueAsDouble();
        inputs.driveAppliedVolts = driveAppliedVoltage.getValueAsDouble();
        inputs.driveTempCelsius = driveMotorTemp.getValueAsDouble();
        inputs.driveStatorCurrentAmps = driveMotorStatorCurrent.getValueAsDouble();
        inputs.drivePositionRads = drivePosition.getValueAsDouble();
        inputs.driveVelocityRadPerSec = driveVelocity.getValueAsDouble();

    }

}
