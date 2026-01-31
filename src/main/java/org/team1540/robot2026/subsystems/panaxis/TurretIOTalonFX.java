package org.team1540.robot2026.subsystems.panaxis;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import jdk.jshell.Snippet;

import static org.team1540.robot2026.subsystems.panaxis.TurretConstants.DRIVE_ID;

public class TurretIOTalonFX {
    // Drive Motor
    private final TalonFX driveMotor = new TalonFX(DRIVE_ID);
    private final StatusSignal<AngularVelocity> driveVelocity = driveMotor.getVelocity();
    private final StatusSignal<Angle> drivePosition = driveMotor.getPosition();
    private final StatusSignal<Voltage> driveAppliedVoltage = driveMotor.getMotorVoltage();
    private final StatusSignal<Current> driveMotorSupplyCurrent = driveMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> driveMotorTemp = driveMotor.getDeviceTemp();
    private final StatusSignal<Current> driveMotorStatorCurrent = driveMotor.getStatorCurrent();

    // CANcoder 1

    // CANcoder 2
}
