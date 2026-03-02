package org.team1540.robot2026.subsystems.PieceDetection;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class Centroid {

    private double x;
    private double y;
    private final List<Piece> pieces = new ArrayList<>();

    public Centroid(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() { return x; }
    public double getY() { return y; }

    public void setPosition(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void clearPieces() {
        pieces.clear();
    }

    public void addPiece(Piece piece) {
        pieces.add(piece);
    }

    public List<Piece> getPieces() {
        return pieces;
    }
    public Translation2d getAsTranslation2d(){
        return new Translation2d(x,y);
    }
    public double getDistance(){
        return Math.sqrt(x*x+y*y);
    }
    public int getNumPieces(){
        return pieces.size();
    }
}