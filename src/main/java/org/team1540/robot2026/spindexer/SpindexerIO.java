package org.team1540.robot2026.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    @AutoLog
    class SpindexerIOInputs {
        public boolean spinMotorConnected = false;
        public double spinVelocityRPS = 0.0;
        public double spinAppliedVolts = 0.0;
        public double spinSupplyCurrentAmps = 0.0;
        public double spinStatorCurrentAmps = 0.0;
        public double spinTempCelsius = 0.0;

        public boolean feederMotorConnected = false;
        public double feederVelocityRPS = 0.0;
        public double feederAppliedVolts = 0.0;
        public double feederSupplyCurrentAmps = 0.0;
        public double feederStatorCurrentAmps = 0.0;
        public double feederTempCelsius = 0.0;
    }

    default void updateInputs(SpindexerIOInputs inputs) {}

    default void setMotorVoltages(double spinVolts, double feederVolts) {}
}
