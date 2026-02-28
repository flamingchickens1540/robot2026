package org.team1540.robot2026.subsystems.PieceDetection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2026.util.LoggedTunableNumber;

import java.util.ArrayList;

public class PieceDetection extends SubsystemBase {
    public PieceDetectionIO detectionIO;

    private final LoggedTunableNumber kp = new LoggedTunableNumber("PieceDetection/kp", 30.0);
    private final LoggedTunableNumber ki = new LoggedTunableNumber("PieceDetection/ki", 30.0);
    private final LoggedTunableNumber kd = new LoggedTunableNumber("PieceDetection/kd", 30.0);
    private final LoggedTunableNumber mode = new LoggedTunableNumber("PieceDetection/useRobotRelative(0:center biggest cluster 1: biggest cluster, 2: closest cluster, 3: use weights as heuristics, else: disable)", 1);
    private final LoggedTunableNumber eachBallPoint = new LoggedTunableNumber("# ball wight");
    private final LoggedTunableNumber farthestBallDistancePointsM = new LoggedTunableNumber("the point value each meter of additional distance takes");

    PIDController pid = new PIDController(kp.get(), ki.get(), kd.get());
// detect fighting give in

    public PieceDetection() {
        detectionIO = new PieceDetectionIOReal();
    }

    public double getPieceDetectionAngle(double kp, double ki, double kd){
        double targetAngle = 0;
            switch ((int) mode.getAsDouble()){
                case 0:
                    // center biggest cluster
                    // if there is cluster within x distance than use biggest one
                    // else pick cluster closest to center
                    //'center' can be changed based off of vel
                    break;
                case 1 :





                    //biggest cluster

                    Translation3d[] detectionsb =  detectionIO.getPoses();

                    // turn detections into an array of doubles
                    ArrayList<double[]> pointsb = new ArrayList<double[]>();

                    for (int i = 0; i < detectionsb.length; i++) {
                        Translation3d detection = detectionsb[i];
                        if (!(detection.getZ() >= 10)) {// if greater than or equal to 10 cm above the ground we don't want it
                            pointsb.add(new double[]{detection.getX(), detection.getY()});
                        }
                        // turn detections into an array of doubles
                    }

                        // loop through find best value for k
                        double bestWCSSb = Double.POSITIVE_INFINITY;
                        KMeans bestMeanb = null;
                        for (int j = 1; j < detectionsb.length-1; j++) {
                            KMeans means = new KMeans(new KMeans.Builder(j, pointsb.toArray(new double[][]{})));
                            if (means.getWCSS()<bestWCSSb){
                                bestMeanb = means;
                                bestWCSSb = means.getWCSS();
                            }
                        }
                       // turn the centroid into an angle to move
                        int largestClusterIndex = bestMeanb.getLargestClusterIndex();
                        double[] centroidWithMostThingsOnIt = bestMeanb.getCentroids()[largestClusterIndex];
                        ArrayList<double[]> pointsForCentroidBiggest = KMeans.getPointsForCentroid(bestMeanb.getAssignment(), pointsb.toArray(new double[][]{}) ,largestClusterIndex);
                        for (Translation3d detection: detectionsb){ // loop through and see if any of the cords match
                            if (pointsForCentroidBiggest.contains(new double[]{detection.getX(), detection.getY()})) targetAngle = Rotation2d.fromRadians(Math.atan2(detection.getY(), detection.getX())).getDegrees();
                        }





                    break;
                case 2 :




                    //closest cluster
                    // l for closest
                    Translation3d[] detectionsl =  detectionIO.getPoses();
                    // turn detections into an array of doubles
                    ArrayList<double[]> pointsl = new ArrayList<double[]>();
                    for (int i = 0; i < detectionsl.length; i++) {
                        Translation3d detection = detectionsl[i];
                        if (!(detection.getZ() >= 10)) {// if greater than or equal to 10 cm above the ground we don't want it
                            pointsl.add(new double[]{detection.getX(), detection.getY()});
                        }
                        // turn detections into an array of doubles
                    }
                        // loop through find best value for k
                        double bestWCSS = Double.POSITIVE_INFINITY;
                        KMeans bestMean = null;
                        for (int j = 1; j < detectionsl.length-1; j++) {
                            KMeans means = new KMeans(new KMeans.Builder(j, pointsl.toArray(new double[][]{})));
                            if (means.getWCSS()<bestWCSS){
                                bestMean = means;
                                bestWCSS = means.getWCSS();
                            }
                        }
                        bestMean.getAssignment();





                    break;
                case 3:
                    //use weights as heuristics
                    break;

            }









        return pid.calculate(0, targetAngle);

    }
}
