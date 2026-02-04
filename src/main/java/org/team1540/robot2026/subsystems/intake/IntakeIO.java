package org.team1540.robot2026.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        public double spinMotorVelocityRPS = 0;
        public double spinMotorAppliedVolts = 0;
        public double spinSupplyCurrentAmps = 0;
        public double spinStatorCurrentAmps = 0;
        public boolean spinConnected = true;

        public boolean pivotConnected = true;
        public Rotation2d pivotPosition = Rotation2d.kZero;
        public double pivotMotorVelocityRPS = 0;
        public double pivotMotorAppliedVolts = 0;
        public double pivotSupplyCurrentAmps = 0;
        public double pivotStatorCurrentAmps = 0;
    }

    default void updateInputs(IntakeInputs inputs) {}

    default void setIntakeVoltage(double voltage) {}

    default void setPivotPosition(Rotation2d pivotPosition) {}

    default void setPivotAppliedVoltage(double voltage) {}
}
