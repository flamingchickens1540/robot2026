package org.team1540.robot2026.subsystems.PieceDetection;

public class Piece {
    private final double x;
    private final double y;
    private final LimelightHelpers.RawDetection savedRawDetection;
    public Piece(LimelightHelpers.RawDetection detection){
        x = ((detection.corner0_X-detection.corner1_X)/2)+detection.corner0_X;
        y = ((detection.corner0_Y-detection.corner1_Y)/2)+detection.corner0_Y;
        savedRawDetection = detection;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public LimelightHelpers.RawDetection getSavedRawDetection() {
        return savedRawDetection;
    }
}
