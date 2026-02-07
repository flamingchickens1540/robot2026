package org.team1540.robot2026.subsystems.climber;

import static org.team1540.robot2026.subsystems.climber.ClimberConstants.*;
import static org.team1540.robot2026.Constants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberIOSim implements ClimberIO {
    private static final double SIM_KP = 300;
    private static final double SIM_KI = 0;
    private static final double SIM_KD = 0;
    private static final double SIM_KS = 0.03178;
    private static final double SIM_KV = 2.562;
    private static final double SIM_KG = 0;
    private final ElevatorSim climberSim = new ElevatorSim(
        DCMotor.getKrakenX60(2),
        GEAR_RATIO,
        SIM_CARRIAGE_MASS_KG,
        SPROCKET_RADIUS_M,
        MIN_HEIGHT_M,
        MAX_HEIGHT_M,
        false,
        MIN_HEIGHT_M
    );
    private double appliedVolts = 0.0;
    private final ProfiledPIDController controller = new ProfiledPIDController(
            SIM_KP, SIM_KI, SIM_KD, new TrapezoidProfile.Constraints(CRUISE_VELOCITY_MPS, ACCELERATION_MPS2));
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(SIM_KS, SIM_KG, SIM_KV);
    private boolean isClosedLoop;
    private TrapezoidProfile.State setpoint;

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        if (isClosedLoop) {
            appliedVolts = controller.calculate(climberSim.getPositionMeters(), setpoint)
                    + feedforward.calculate(controller.getSetpoint().velocity);
        }

        climberSim.setInputVoltage(appliedVolts);
        climberSim.update(LOOP_PERIOD_SECS);

        inputs.leftMotorConnected = true;
        inputs.leftMotorPosition = climberSim.getPositionMeters();
        inputs.leftMotorVelocityRPM = climberSim.getVelocityMetersPerSecond();
        inputs.leftMotorAppliedVolts = appliedVolts;
        inputs.leftMotorSupplyCurrentAmps = climberSim.getCurrentDrawAmps();
        inputs.leftMotorStatorCurrentAmps = climberSim.getCurrentDrawAmps();

        inputs.rightMotorConnected = true;
        inputs.rightMotorPosition = climberSim.getPositionMeters();
        inputs.rightMotorVelocityRPM = climberSim.getVelocityMetersPerSecond();
        inputs.rightMotorAppliedVolts = appliedVolts;
        inputs.rightMotorSupplyCurrentAmps = climberSim.getCurrentDrawAmps();
        inputs.rightMotorStatorCurrentAmps = climberSim.getCurrentDrawAmps();

        inputs.atUpperLimit = climberSim.hasHitUpperLimit();
        inputs.atLowerLimit = climberSim.hasHitLowerLimit();
    }

    @Override
    public void setSetpoint(double setpointMeters) {
        isClosedLoop = true;
        setpoint = new TrapezoidProfile.State(setpointMeters, 0.0);
    }

    @Override
    public void setVoltage(double volts) {
        isClosedLoop = false;
        appliedVolts = volts;
    }
}
