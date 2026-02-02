package org.team1540.robot2026.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public boolean leftMotorConnected = true;
        public Rotation2d leftMotorPosPosition = new Rotation2d();
        public double leftMotorVelocityRPM = 0;
        public double leftMotorAppliedVolts = 0;
        public double leftMotorSupplyCurrentAmps = 0;
        public double leftMotorStatorCurrentAmps = 0;
        public double leftMotorTempC = 0;

        public boolean rightMotorConnected = true;
        public Rotation2d rightMotorPosPosition = new Rotation2d();
        public double rightMotorVelocityRPM = 0;
        public double rightMotorAppliedVolts = 0;
        public double rightMotorSupplyCurrentAmps = 0;
        public double rightMotorStatorCurrentAmps = 0;
        public double rightMotorTempC = 0;
    }

    default void updateInputs(ClimberIOInputs inputs) {}

    default void setVoltage(double voltage) {}

    default void setPosition(Rotation2d rotation) {}

    default void configPID(double kP, double kI, double kD) {}

    default void configFF(double kS, double kV, double kG) {}

    default void setBrakeMode(boolean setBrake) {}
} 