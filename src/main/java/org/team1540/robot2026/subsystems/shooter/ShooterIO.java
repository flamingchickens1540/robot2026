package org.team1540.robot2026.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {
        public boolean leftMotorConnected = false;
        public double leftAppliedVolts = 0.0;
        public double leftStatorCurrentAmps = 0.0;
        public double leftSupplyCurrentAmps = 0.0;
        public double leftVelocityRPM = 0.0;
        public double leftTempCelsius = 0.0;

        public boolean rightMotorConnected = false;
        public double rightAppliedVolts = 0.0;
        public double rightStatorCurrentAmps = 0.0;
        public double rightSupplyCurrentAmps = 0.0;
        public double rightVelocityRPM = 0.0;
        public double rightTempCelsius = 0.0;
    }

    /**
     * Updates the set of loggable inputs
     */
    default void updateInputs(ShooterIOInputs inputs) {}

    /**
     * Runs open loop at the specified voltage
     */
    default void setVoltage(double volts) {}

    /**
     * Runs closed loop at the specified RPM
     */
    default void setSpeed(double rpm) {}

    /**
     * Configures the PID controller
     */
    default void configPID(double kP, double kI, double kD, double kS, double kV) {}
}
