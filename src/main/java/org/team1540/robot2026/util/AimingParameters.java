package org.team1540.robot2026.util;

import edu.wpi.first.math.geometry.Rotation2d;

public record AimingParameters(
        Rotation2d turretAngle, double turretVelocityRadPerSec, Rotation2d hoodAngle, double shooterRPM) {}
