package org.team1540.robot2026.subsystems.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SpindexerIOSim implements SpindexerIO {
    private final DCMotorSim intakeSim = new DCMotorSim(DCMotor.getNEO(1), INTAKE_GEAR_RATIO, INTAKE_MOI);
    private final DCMotorSim feederSim = new DCMotorSim(DCMotor.getNEO(1), FEEDER_GEAR_RATIO, FEEDER_MOI);
}
