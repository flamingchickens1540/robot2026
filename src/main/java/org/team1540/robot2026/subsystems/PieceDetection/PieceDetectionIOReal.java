package org.team1540.robot2026.subsystems.PieceDetection;

import edu.wpi.first.math.geometry.*;
import org.team1540.robot2026.subsystems.vision.AprilTagVisionIO;

public class PieceDetectionIOReal implements PieceDetectionIO {
    private static final String limelightName = "limelight-bill"; // don't ask me why it's called that I didn't name it

    public Translation3d[] getPoses() {
        LimelightHelpers.RawDetection[] detections = LimelightHelpers.getRawDetections(limelightName);
        Translation3d[] poses = new Translation3d[detections.length];
        int index = 0;
        for (LimelightHelpers.RawDetection rawDetections : detections) {

            double distanceCm = get_distance_cm(rawDetections);

            // magical math I don't understand
            double yawRad = Math.toRadians(rawDetections.txnc);

            double pitchRad = Math.toRadians(rawDetections.tync);

            double x = distanceCm * Math.cos(pitchRad) * Math.cos(yawRad);

            double y = distanceCm * Math.cos(pitchRad) * Math.sin(yawRad);

            double z = distanceCm * Math.sin(pitchRad);

            poses[index] = new Translation3d(x, y, z).plus(PieceDetectionConstants.CAMERA_POS); // add to arrayList
            index ++;
        }
        return poses;
    }
    public LimelightHelpers.RawDetection[] getBoundingBoxes(){
        return LimelightHelpers.getRawDetections(limelightName);
    }
    private double get_distance_cm(LimelightHelpers.RawDetection rawDetection){
        double ballDiameterPixels = (Math.abs(rawDetection.corner1_X - rawDetection.corner0_X)
                + Math.abs(rawDetection.corner2_Y - rawDetection.corner0_Y))
                / 2;
        return (PieceDetectionConstants.FUEL_DIAMETER_CM * PieceDetectionConstants.FOCAL_LENGTH_CM)
                / ballDiameterPixels;
    }

    @Override
    public void updateInputs(PieceDetectionIOInputs inputs) {
        inputs.fuelCount = LimelightHelpers.getTargetCount(limelightName);
        inputs.lattencyMills = LimelightHelpers.getLatency_Capture(limelightName);
        inputs.fuelsPoses = getPoses();
    }
}
