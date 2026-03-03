package org.team1540.robot2026.subsystems.PieceDetection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Triangle;
import org.dyn4j.geometry.Vector2;
import org.team1540.robot2026.RobotContainer;
import org.team1540.robot2026.subsystems.drive.Drivetrain;
import org.team1540.robot2026.util.LoggedTunableNumber;

import java.util.ArrayList;

public class PieceDetection extends SubsystemBase {
    public PieceDetectionIO detectionIO;

    private final LoggedTunableNumber kp = new LoggedTunableNumber("PieceDetection/kp", 30.0);
    private final LoggedTunableNumber ki = new LoggedTunableNumber("PieceDetection/ki", 30.0);
    private final LoggedTunableNumber kd = new LoggedTunableNumber("PieceDetection/kd", 30.0);
    private final LoggedTunableNumber mode = new LoggedTunableNumber("PieceDetection/useRobotRelative(0:center biggest cluster 1:center closest cluster, 2: biggest cluster, 3: closest cluster, 4: use weights as heuristics, 5: brute force best path (do not use in match is very unstable may cause robot to freeze for several minutes), else: disable)", 1);
    private final LoggedTunableNumber eachBallPoint = new LoggedTunableNumber("# ball wight");
    private final LoggedTunableNumber farthestBallDistancePointsM = new LoggedTunableNumber("the point value each meter of additional distance takes");
    private final LoggedTunableNumber coneLength = new LoggedTunableNumber("cone length 4 center calculations");

    PIDController pid = new PIDController(kp.get(), ki.get(), kd.get());
// detect fighting give in

    public PieceDetection() {
        detectionIO = new PieceDetectionIOReal();
    }

    public double getPieceDetectionAngle(double currentRotationalVelocity){
        double targetAngle = 0;
            switch ((int) mode.getAsDouble()){
                case 0: {
                    // center biggest cluster
                    // if there is cluster within x distance than use the biggest one
                    Translation3d[] detections = detectionIO.getPoses();
                    Triangle triangle = new Triangle(new Vector2(0, coneLength.getAsDouble() / 2), new Vector2(0, -coneLength.getAsDouble() / 2), new Vector2());
                    triangle.rotate(currentRotationalVelocity);
                    ArrayList<Piece> pieces = new ArrayList<>();
                    for (Translation3d translation3d : detections) {
                        pieces.add(new Piece(translation3d.toTranslation2d()));
                    }
                    //remove detections not in the triangle
                    for (Piece piece:pieces){
                        if (!triangle.contains(new Vector2(piece.getX(), piece.getY()), new Transform())) pieces.remove(piece);
                    }
                    // make sure there are balls

                    if (detections.length > 1) {
                        double bestWCSS = Double.POSITIVE_INFINITY;
                        KMeans bestKMeans = null;

                        for (int i = 1; i < pieces.size() - 1; i++) {
                            KMeans kMean = new KMeans(i, pieces, 5, 0.1);
                            if (kMean.getWCSS() < bestWCSS) {// smaller WCSS is better WCSS describes how close balls are t o each-other
                                bestWCSS = kMean.getWCSS();
                                bestKMeans = kMean;
                            }
                        }
                        assert bestKMeans != null;// this should not be possible if the WCSS is in the millions we have a huge problem
                        Translation2d best = bestKMeans.getLargestCluster().getAsTranslation2d();
                        targetAngle = Math.atan2(best.getY(), best.getX());
                    } else if (detections.length==1) {
                        Translation2d best = detections[0].toTranslation2d();
                        targetAngle = Math.atan2(best.getY(), best.getX());
                    }else {
                        targetAngle=0;
                    }

                }
                    // else pick cluster closest to center
                    //'center' can be changed based off of vel
                    break;
                case 1:{
                    // center biggest cluster
                    // if there is cluster within x distance than use the biggest one
                    Translation3d[] detections = detectionIO.getPoses();
                    Triangle triangle = new Triangle(new Vector2(0, coneLength.getAsDouble() / 2), new Vector2(0, -coneLength.getAsDouble() / 2), new Vector2());
                    triangle.rotate(currentRotationalVelocity);
                    ArrayList<Piece> pieces = new ArrayList<>();
                    for (Translation3d translation3d : detections) {
                        pieces.add(new Piece(translation3d.toTranslation2d()));
                    }
                    //remove detections not in the triangle
                    for (Piece piece:pieces){
                        if (!triangle.contains(new Vector2(piece.getX(), piece.getY()), new Transform())) pieces.remove(piece);
                    }
                    // make sure there are balls

                    if (detections.length > 1) {
                        double bestWCSS = Double.POSITIVE_INFINITY;
                        KMeans bestKMeans = null;

                        for (int i = 1; i < pieces.size() - 1; i++) {
                            KMeans kMean = new KMeans(i, pieces, 5, 0.1);
                            if (kMean.getWCSS() < bestWCSS) {// smaller WCSS is better WCSS describes how close balls are t o each-other
                                bestWCSS = kMean.getWCSS();
                                bestKMeans = kMean;
                            }
                        }
                        double smallestDistance = Double.POSITIVE_INFINITY;
                        Centroid centroidWithSmallestDistance = null;
                        for (Centroid centroid: bestKMeans.getCentroids()){
                            if (centroid.getDistance()<smallestDistance){
                                smallestDistance = Math.atan2(centroid.getX(), centroid.getY());
                                centroidWithSmallestDistance = centroid;
                            }
                        }
                        assert centroidWithSmallestDistance != null; //yeah the closest cluster being millions away doesn't make sense
                        targetAngle = Math.atan2(centroidWithSmallestDistance.getY(), centroidWithSmallestDistance.getX());
                    } else if (detections.length==1) {
                        Translation2d best = detections[0].toTranslation2d();
                        targetAngle = Math.atan2(best.getY(), best.getX());
                    }else {
                        targetAngle=0;
                    }

                }
                case 2 : {


                    //biggest cluster

                    Translation3d[] detections = detectionIO.getPoses();
                    if (detections.length > 1) {
                        ArrayList<Piece> pieces = new ArrayList<>();
                        for (Translation3d translation3d : detections) {
                            pieces.add(new Piece(translation3d.toTranslation2d()));

                        }
                        double bestWCSS = Double.POSITIVE_INFINITY;
                        KMeans bestKMeans = null;

                        for (int i = 1; i < pieces.size() - 1; i++) {
                            KMeans kMean = new KMeans(i, pieces, 5, 0.1);
                            if (kMean.getWCSS() < bestWCSS) {// smaller WCSS is better WCSS describes how close balls are t o each-other
                                bestWCSS = kMean.getWCSS();
                                bestKMeans = kMean;
                            }
                        }
                        assert bestKMeans != null;// this should not be possible if the WCSS is in the millions we have a huge problem
                        Translation2d best = bestKMeans.getLargestCluster().getAsTranslation2d();
                        targetAngle = Math.atan2(best.getY(), best.getX());
                    } else if (detections.length == 1) { // if there is only one ball we can see just go to that one
                        Translation2d best = detections[0].toTranslation2d();
                        targetAngle = Math.atan2(best.getY(), best.getX());
                    } else {
                        targetAngle = 0;
                    }
                }


                    break;
                case 3 : {


                    //closest cluster

                    Translation3d[] detections = detectionIO.getPoses();
                    if (detections.length > 1) {
                        ArrayList<Piece> pieces = new ArrayList<>();
                        for (Translation3d translation3d : detections) {
                            pieces.add(new Piece(translation3d.toTranslation2d()));
                        }
                        double bestWCSS = Double.POSITIVE_INFINITY;
                        KMeans bestKMeans = null;

                        for (int i = 1; i < pieces.size() - 1; i++) {
                            KMeans kMean = new KMeans(i, pieces, 5, 0.1);

                            if (kMean.getWCSS() < bestWCSS) {// smaller WCSS is better WCSS describes how close balls are t o each-other
                                bestWCSS = kMean.getWCSS();
                                bestKMeans = kMean;
                            }
                        }
                        assert bestKMeans != null;// this should not be possible if the WCSS is in the millions we have a huge problem
                        double smallestDistance = Double.POSITIVE_INFINITY;
                        Centroid centroidWithSmallestDistance = null;
                        for (Centroid centroid: bestKMeans.getCentroids()){
                            if (centroid.getDistance()<smallestDistance){
                                smallestDistance = Math.atan2(centroid.getX(), centroid.getY());
                                centroidWithSmallestDistance = centroid;
                            }
                        }
                        assert centroidWithSmallestDistance != null; //yeah the closest cluster being millions away doesn't make sense
                        targetAngle = Math.atan2(centroidWithSmallestDistance.getY(), centroidWithSmallestDistance.getX());

                    } else if (detections.length == 1) { // if there is only one ball we can see just go to that one
                        Translation2d best = detections[0].toTranslation2d();
                        targetAngle = Math.atan2(best.getY(), best.getX());
                    } else {
                        targetAngle = 0;
                    }

                }


                    break;
                case 4:



                    //use weights as heuristics



                    Translation3d[] detections = detectionIO.getPoses();
                    if (detections.length > 1) {
                        ArrayList<Piece> pieces = new ArrayList<>();
                        for (Translation3d translation3d : detections) {
                            pieces.add(new Piece(translation3d.toTranslation2d()));
                        }
                        double bestWCSS = Double.POSITIVE_INFINITY;
                        KMeans bestKMeans = null;

                        for (int i = 1; i < pieces.size() - 1; i++) {
                            KMeans kMean = new KMeans(i, pieces, 5, 0.1);

                            if (kMean.getWCSS() < bestWCSS) {// smaller WCSS is better WCSS describes how close balls are t o each-other
                                bestWCSS = kMean.getWCSS();
                                bestKMeans = kMean;
                            }
                        }
                        //implement deciding which centroid is the best
                            Centroid bestCentroid = null;
                            double bestScore = Double.MIN_VALUE;
                            double score;
                            for (Centroid centroid:bestKMeans.getCentroids()){
                                score=0;
                                bestScore+= centroid.getNumPieces()*eachBallPoint.getAsDouble();
                                bestScore-= centroid.getDistance()* farthestBallDistancePointsM.getAsDouble();
                                if (score > bestScore){
                                    bestScore = score;
                                    bestCentroid=centroid;
                                }
                            }
                            assert bestCentroid != null;
                        targetAngle = Math.atan2(bestCentroid.getY(), bestCentroid.getX());

                    } else if (detections.length == 1) { // if there is only one ball we can see just go to that one
                        Translation2d best = detections[0].toTranslation2d();
                        targetAngle = Math.atan2(best.getY(), best.getX());
                    } else {
                        targetAngle = 0;
                    }
                    break;

            }


        return pid.calculate(0, targetAngle);

    }
}
