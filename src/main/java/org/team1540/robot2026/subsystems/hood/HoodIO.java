package org.team1540.robot2026.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    class HoodIOInputs {
        public boolean motorConnected = false;

        public Rotation2d position = Rotation2d.kZero;
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(HoodIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setSetpoint(Rotation2d position) {}

    default void resetPosition(Rotation2d position) {}

    default void setBrakeMode(boolean enabled) {}

    default void configPID(double kP, double kI, double kD, double kS, double kV, double kG) {}
}
