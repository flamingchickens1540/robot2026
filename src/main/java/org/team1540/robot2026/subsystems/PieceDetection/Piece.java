package org.team1540.robot2026.subsystems.PieceDetection;

import edu.wpi.first.math.geometry.Translation2d;

public class Piece {
    private final double x;
    private final double y;
    private final Object savedRawDetection;
    private final boolean isRawDetection;

    public Piece(LimelightHelpers.RawDetection detection) {
        x = ((detection.corner0_X - detection.corner1_X) / 2) + detection.corner0_X;
        y = ((detection.corner0_Y - detection.corner1_Y) / 2) + detection.corner0_Y;
        savedRawDetection = detection;
        isRawDetection = true;
    }

    public Piece(Translation2d translation2d) {
        x = translation2d.getX();
        y = translation2d.getY();
        isRawDetection = false;
        savedRawDetection = translation2d;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public LimelightHelpers.RawDetection getSavedRawDetection() {
        if (isRawDetection) {
            return (LimelightHelpers.RawDetection) savedRawDetection;
        } else {
            return null;
        }
    }

    public Translation2d getSavedPosition() {
        if (!isRawDetection) {
            return (Translation2d) savedRawDetection;
        } else {
            return null;
        }
    }
}
