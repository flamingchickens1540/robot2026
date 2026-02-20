package org.team1540.robot2026.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    default void configFF(double kS, double kV) {}
    ;

    @AutoLog
    class TurretIOInputs {
        //  Motor
        public boolean connected = false;
        public Rotation2d position = Rotation2d.kZero;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;

        // Cancoder Main
        public boolean gear1EncoderConnected = false;
        public Rotation2d gear1EncoderPosition = Rotation2d.kZero;

        // Cancoder Second
        public boolean gear2EncoderConnected = false;
        public Rotation2d gear2EncoderPosition = Rotation2d.kZero;
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setSetpoint(Rotation2d rotation) {}

    default void setBrakeMode(boolean brakeMode) {}

    default void configPID(double kP, double kI, double kD) {}

    default void configFF(double kS, double kV, double kG) {}

    default void setMotorPosition(Rotation2d position) {}

    default double getGear1EncoderPos() {
        return 0;
    }

    default double getGear2EncoderPos() {
        return 0;
    }
}
