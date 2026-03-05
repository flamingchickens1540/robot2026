package org.team1540.robot2026.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public boolean motorConnected = true;
        public double positionMeters = 0;
        public double velocityMPS = 0;
        public double appliedVolts = 0;
        public double supplyCurrentAmps = 0;
        public double statorCurrentAmps = 0;
        public double tempCelsius = 0;

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
