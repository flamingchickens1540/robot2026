package org.team1540.robot2026.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {
        public boolean leaderConnected = false;
        public boolean followerConnected = false;
        public double[] appliedVolts = new double[] {0.0, 0.0};
        public double[] statorCurrentAmps = new double[] {0.0, 0.0};
        public double[] supplyCurrentAmps = new double[] {0.0, 0.0};
        public double[] velocityRPM = new double[] {0.0, 0.0};
        public double[] tempCelsius = new double[] {0.0, 0.0};
    }

    /**
     * Updates the set of loggable inputs
     */
    default void updateInputs(ShooterIOInputs inputs) {}

    /**
     * Runs open loop at the specified voltages
     */
    default void setVoltage(double volts) {}

    /**
     * Runs closed loop at the specified RPMs
     */
    default void runVelocity(double velocityRPM) {}

    /**
     * Configures the PID controller
     */
    default void configPID(double kP, double kI, double kD, double kS, double kV) {}
}
