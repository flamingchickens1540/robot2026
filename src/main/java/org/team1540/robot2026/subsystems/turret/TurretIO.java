package org.team1540.robot2026.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public boolean connected = false;
        public Rotation2d position = Rotation2d.kZero;
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;

        public boolean smallEncoderConnected = false;
        public Rotation2d smallEncoderPosition = Rotation2d.kZero;

        public boolean bigEncoderConnected = false;
        public Rotation2d bigEncoderPosition = Rotation2d.kZero;
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setSetpoint(Rotation2d rotation, double voltageFF) {}

    default void setMotorPosition(Rotation2d position) {}

    default void setBrakeMode(boolean brakeMode) {}

    default void configPID(double kP, double kI, double kD) {}

    default void configFF(double kS, double kV) {}
}
