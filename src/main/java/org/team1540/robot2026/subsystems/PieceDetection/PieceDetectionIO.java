package org.team1540.robot2026.subsystems.PieceDetection;

import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.AutoLog;

public interface PieceDetectionIO {
    @AutoLog
    class PieceDetectionIOInputs {
        public double lattencyMills = 0;
        public int fuelCount = 0;
        public Translation3d[] fuelsPoses;
    }

    default void updateInputs(PieceDetectionIOInputs inputs) {}
    LimelightHelpers.RawDetection[] getBoundingBoxes();
    Translation3d[] getPoses();
}
