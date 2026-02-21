package org.team1540.robot2026.subsystems.PieceDetection;

import org.littletonrobotics.junction.AutoLog;
import org.team1540.robot2026.subsystems.Shooter.FlywheelsIO;

public interface PieceDetectionIO {
    @AutoLog
    class PieceDetectionIOInputs {
        public double lattencyMills = 0;
        public int fuelCount = 0;
    }
    default void updateInputs(PieceDetectionIOInputs inputs) {}

}
