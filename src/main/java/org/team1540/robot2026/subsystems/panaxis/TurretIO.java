package org.team1540.robot2026.subsystems.panaxis;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        // Drive Motor
        public boolean driveConnected = false;
        public double drivePositionRads = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveSupplyCurrentAmps = 0.0;
        public double driveStatorCurrentAmps = 0.0;
        public double driveTempCelsius = 0.0;

        // Cancoder Extra
        public boolean driveEncoderConnected = false;
        public Rotation2d driveEncoderAbsolutePosition = Rotation2d.kZero;

        // Cancoder Extra
        public boolean extraEncoderConnected = false;
        public Rotation2d extraEncoderAbsolutePosition = Rotation2d.kZero;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryEncoderPositionsRads = new double[] {};
    }
    default void updateInputs(TurretIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setRotation(double rotation) {}

    default void setBrakeMode(boolean brakeMode) {}

    default void configPID(double kP, double kI, double kD) {}

    default void configFF(double kS, double kV, double kG) {}
}
