package org.team1540.robot2026.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public boolean leftMotorConnected = true;
        public double leftMotorPositionMeters = 0;
        public double leftMotorVelocityMPS = 0;
        public double leftMotorAppliedVolts = 0;
        public double leftMotorSupplyCurrentAmps = 0;
        public double leftMotorStatorCurrentAmps = 0;
        public double leftMotorTempC = 0;

        public boolean rightMotorConnected = true;
        public double rightMotorPositionMeters = 0;
        public double rightMotorVelocityMPS = 0;
        public double rightMotorAppliedVolts = 0;
        public double rightMotorSupplyCurrentAmps = 0;
        public double rightMotorStatorCurrentAmps = 0;
        public double rightMotorTempC = 0;

        public boolean atUpperLimit = false;
        public boolean atLowerLimit = false;
    }

    default void updateInputs(ClimberIOInputs inputs) {}

    default void setVoltage(double voltage) {}

    default void setSetpoint(double setpointMeters) {}

    default void configPID(double kP, double kI, double kD) {}

    default void configFF(double kS, double kV, double kG) {}

    default void setBrakeMode(boolean setBrake) {}
}
