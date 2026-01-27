package org.team1540.robot2026.subsystems.panaxis;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    class DriveIOInputs {
        // Drive Motor
        public boolean driveConnected = false;
        public double drivePositionRads = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveSupplyCurrentAmps = 0.0;
        public double driveStatorCurrentAmps = 0.0;
        public double driveTempCelsius = 0.0;
        // Cancoder
        public boolean driveEncoderConnected = false;
        public Rotation2d driveEncoderAbsolutePosition = Rotation2d.kZero;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRads = new double[] {};
    }
    }
}
