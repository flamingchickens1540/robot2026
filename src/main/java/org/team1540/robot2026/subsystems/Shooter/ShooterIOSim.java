/*
package org.team1540.robot2026.subsystems.Shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static org.team1540.robot2026.subsystems.Shooter.ShooterConstants.Flywheels.GEAR_RATIO;

public class FlywheelsIOSim implements FlywheelsIO {
        private static final double SIM_KP = 0;
        private static final double SIM_KI = 0;
        private static final double SIM_KD = 0;
        private static final double SIM_KS = 0;
        private static final double SIM_KV = 0;
        private static final double SIM_KG = 0;

     private final FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getKrakenX60(1), GEAR_RATIO));
     private final boolean isClosedLoop;
     private double appliedVolts = 0.0;

     @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.velocityRPM = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.tempCelsius = temperature.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    }
    @Override
    public void setSpeeds(double RPM){


    }
}

*/
