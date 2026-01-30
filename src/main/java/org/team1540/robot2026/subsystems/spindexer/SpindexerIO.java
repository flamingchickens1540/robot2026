package org.team1540.robot2026.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    @AutoLog
    class SpindexerIOInputs {
        public double intakeVoltage = 0.0;
        public double intakeCurrentAmps = 0.01;
        public double intakeVelocityRPM = 0.0;
        public double intakeTempCelsius = 0.0;

        public double feederVoltage = 0.0;
        public double feederCurrentAmps = 0.0;
        public double feederVelocityRPM = 0.0;
        public double feederTempCelsius = 0.0;
        public boolean noteInSpindexer = false;
        public boolean noteInIntake = false;
        public boolean noteInShooter = false;
    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(SpindexerIOInputs inputs) {}

    default void setIntakeVoltage(double volts) {}

    default void setFeederVoltage(double volts) {}

    default void setFeederVelocity(double velocityRPM) {}

    default void configureFeederPID(double p, double i, double d) {}

    default void setIntakeBrakeMode(boolean isBrakeMode) {}
}
