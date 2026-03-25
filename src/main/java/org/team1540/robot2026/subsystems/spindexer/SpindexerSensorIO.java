package org.team1540.robot2026.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerSensorIO {
    @AutoLog
    class SpindexerSensorIOInputs {
        public double distanceMM = 0.0;
        public boolean isConnected = false;
    }

    default void updateInputs(SpindexerSensorIOInputs inputs) {}

    default double getDistanceMM() {
        return 0.0;
    }
}
