package org.team1540.robot2026.subsystems.PieceDetection;

import edu.wpi.first.math.geometry.*;

import java.util.ArrayList;

public class PieceDetectionIOReal implements PieceDetectionIO{
    private static String limelightName = "limelight-bill";// don't ask me why it's called that I didn't name it
    public static ArrayList<Translation3d> getPoses(){
        ArrayList<Translation3d> poses =  new ArrayList();
        for (LimelightHelpers.RawDetection rawDetections: LimelightHelpers.getRawDetections(limelightName)){
            double ballDiameterPixels = (Math.abs(rawDetections.corner1_X - rawDetections.corner0_X)+Math.abs(rawDetections.corner2_Y - rawDetections.corner0_Y))/2;

            double distanceCm = (PieceDetectionConstants.FUEL_DIAMETER_CM * PieceDetectionConstants.FOCAL_LENGTH)/ballDiameterPixels;

            new Rotation3d(0, Rotation2d.fromDegrees(rawDetections.tync).getRadians(), Rotation2d.fromDegrees(rawDetections.txnc).getRadians()); // angle between camera and the line ypu could draw from the camera to the fuel

            double yawRad =
                    Math.toRadians(rawDetections.txnc);

            double pitchRad =
                    Math.toRadians(rawDetections.tync);

            double x =
                    distanceCm * Math.cos(pitchRad) * Math.cos(yawRad);

            double y =
                    distanceCm * Math.cos(pitchRad) * Math.sin(yawRad);

            double z =
                    distanceCm * Math.sin(pitchRad);

            poses.add(new Translation3d(x,y,z));


        }
        return poses;
    }
    @Override
    public void updateInputs(PieceDetectionIOInputs inputs) {
        inputs.fuelCount = LimelightHelpers.getTargetCount(limelightName);
        inputs.lattencyMills = LimelightHelpers.getLatency_Capture(limelightName);
    }
}
