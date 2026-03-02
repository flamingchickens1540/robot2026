package org.team1540.robot2026.subsystems.PieceDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class KMeans {

    private final int k;
    private final List<Piece> pieces;
    private final List<Centroid> centroids = new ArrayList<>();

    private final int maxIterations;
    private final double epsilon;

    public KMeans(int k, List<Piece> pieces, int maxIterations, double epsilon) {
        if (k > pieces.size()) {
            throw new IllegalArgumentException("k cannot exceed number of pieces");
        }

        this.k = k;
        this.pieces = pieces;
        this.maxIterations = maxIterations;
        this.epsilon = epsilon;

        initializeCentroids();
        run();
    }


    // Main algorithm


    private void run() {
        for (int i = 0; i < maxIterations; i++) {

            clearAssignments();
            assignmentStep();

            boolean moved = updateStep();

            if (!moved) break; // centroids stopped moving
        }
    }


    // Initialize centroids randomly


    private void initializeCentroids() {
        Random random = new Random();
        List<Piece> shuffled = new ArrayList<>(pieces);

        for (int i = 0; i < k; i++) {
            Piece p = shuffled.get(random.nextInt(shuffled.size()));
            centroids.add(new Centroid(p.getX(), p.getY()));
        }
    }


    // Assignment Step


    private void assignmentStep() {
        for (Piece piece : pieces) {

            Centroid closest = null;
            double minDist = Double.POSITIVE_INFINITY;

            for (Centroid centroid : centroids) {
                double dist = distance(piece, centroid);

                if (dist < minDist) {
                    minDist = dist;
                    closest = centroid;
                }
            }

            closest.addPiece(piece);
        }
    }


    // Update Step


    private boolean updateStep() {
        boolean moved = false;

        for (Centroid centroid : centroids) {

            if (centroid.getPieces().isEmpty()) continue;

            double sumX = 0;
            double sumY = 0;

            for (Piece piece : centroid.getPieces()) {
                sumX += piece.getX();
                sumY += piece.getY();
            }

            double newX = sumX / centroid.getPieces().size();
            double newY = sumY / centroid.getPieces().size();

            if (Math.abs(newX - centroid.getX()) > epsilon ||
                    Math.abs(newY - centroid.getY()) > epsilon) {

                centroid.setPosition(newX, newY);
                moved = true;
            }
        }

        return moved;
    }

    private void clearAssignments() {
        for (Centroid centroid : centroids) {
            centroid.clearPieces();
        }
    }


    // Distance


    private double distance(Piece p, Centroid c) {
        double dx = p.getX() - c.getX();
        double dy = p.getY() - c.getY();
        return dx * dx + dy * dy; // squared Euclidean (faster, no sqrt)
    }


    // Getters


    public List<Centroid> getCentroids() {
        return centroids;
    }


    public Centroid getLargestCluster() {
        Centroid largest = centroids.get(0);

        for (Centroid c : centroids) {
            if (c.getPieces().size() > largest.getPieces().size()) {
                largest = c;
            }
        }

        return largest;
    }
    public double getWCSS() {
        double wcss = 0.0;

        for (Centroid centroid : centroids) {
            for (Piece piece : centroid.getPieces()) {
                wcss += distance(piece, centroid);
            }
        }

        return wcss;
    }
}