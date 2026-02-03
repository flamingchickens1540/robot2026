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

        // Cancoder 13
        public boolean mainEncoderConnected = false;
        public Rotation2d mainEncoderAbsolutePosition = Rotation2d.kZero;

        // Cancoder 14
        public boolean secondaryEncoderConnected = false;
        public Rotation2d secondaryEncoderAbsolutePosition = Rotation2d.kZero;

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
