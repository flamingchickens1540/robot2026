package org.team1540.robot2026.subsystems.intake;

public interface IntakeIO {
    /* @AutoLog */
    class IntakeInputs {
        public double spinMotorVelocityRPS = 0;
        public double spinMotorAppliedVolts = 0;
        public double spinSupplyCurrentAmps = 0;
        public double spinStatorCurrentAmps = 0;
        public boolean spinConnected = true;
    }
}