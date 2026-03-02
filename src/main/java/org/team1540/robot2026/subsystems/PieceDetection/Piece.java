package org.team1540.robot2026.subsystems.PieceDetection;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class Piece {
    private final double x;
    private final double y;
    private final Object savedRawDetection;
    public Piece(LimelightHelpers.RawDetection detection){
        x = ((detection.corner0_X-detection.corner1_X)/2)+detection.corner0_X;
        y = ((detection.corner0_Y-detection.corner1_Y)/2)+detection.corner0_Y;
        savedRawDetection = detection;
    }
    public Piece(Translation2d translation2d){
        x = translation2d.getX();
        y = translation2d.getY();

        savedRawDetection = translation2d;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public LimelightHelpers.RawDetection getSavedRawDetection() {
        return (LimelightHelpers.RawDetection) savedRawDetection;
    }
}
