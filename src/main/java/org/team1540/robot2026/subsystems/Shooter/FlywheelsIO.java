package org.team1540.robot2026.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {

    @AutoLog
    class FlywheelsIOInputs {
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double velocityRPM = 0.0;
        public double tempCelsius = 0.0;

        public boolean flywheelsConnected = false;
    }

    /**
     * Updates the set of loggable inputs
     */
    default void updateInputs(FlywheelsIOInputs inputs) {}

    /**
     * Runs open loop at the specified voltages
     */
    default void setVoltage(double volts) {}

    /**
     * Runs closed loop at the specified RPMs
     */
    default void setSpeeds(double RPM) {}

    /**
     * Configures the PID controller
     */
    default void configPID(double kP, double kI, double kD, double kV, double kS) {}
}
