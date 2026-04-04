package org.team1540.robot2026.subsystems.spindexer;

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
        public double feeder1VelocityRPS = 0.0;
        public double feeder1AppliedVolts = 0.0;
        public double feeder1SupplyCurrentAmps = 0.0;
        public double feeder1StatorCurrentAmps = 0.0;
        public double feeder1TempCelsius = 0.0;

        public boolean feeder2MotorConnected = false;
        public double feeder2VelocityRPS = 0.0;
        public double feeder2AppliedVolts = 0.0;
        public double feeder2SupplyCurrentAmps = 0.0;
        public double feeder2StatorCurrentAmps = 0.0;
        public double feeder2TempCelsius = 0.0;
    }

    default void updateInputs(SpindexerIOInputs inputs) {}

    default void setMotorVoltages(double spinVolts, double feederVolts, double feeder2Volts) {}
}
