package org.team1540.robot2026.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        public double leftSpinMotorVelocityRPS = 0;
        public double leftSpinMotorAppliedVolts = 0;
        public double leftSpinSupplyCurrentAmps = 0;
        public double leftSpinStatorCurrentAmps = 0;
        public double leftSpinMotorTemp = 0;
        public boolean leftSpinConnected = true;

        public double rightSpinMotorVelocityRPS = 0;
        public double rightSpinMotorAppliedVolts = 0;
        public double rightSpinSupplyCurrentAmps = 0;
        public double rightSpinStatorCurrentAmps = 0;
        public double rightSpinMotorTemp = 0;
        public boolean rightSpinConnected = true;

        public boolean pivotConnected = true;
        public Rotation2d pivotPosition = Rotation2d.kZero;
        public double pivotMotorVelocityRPS = 0;
        public double pivotMotorAppliedVolts = 0;
        public double pivotSupplyCurrentAmps = 0;
        public double pivotStatorCurrentAmps = 0;
        public double pivotMotorTemp = 0;
    }

    default void updateInputs(IntakeInputs inputs) {}

    default void setIntakeVoltage(double voltage) {}

    default void setPivotSetpoint(Rotation2d pivotPosition) {}

    default void resetPivotPosition(Rotation2d pivotPosition) {}

    default void setPivotVoltage(double voltage) {}

    default void setPivotPID(double kP, double kI, double kD) {}

    default void setPivotFF(double kS, double kV, double kG) {}
}
