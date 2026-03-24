package org.team1540.robot2026.subsystems.hood;

import static edu.wpi.first.units.Units.Volts;
import static org.team1540.robot2026.subsystems.hood.HoodConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.SimState;

public class HoodIOSim implements HoodIO {
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            GEAR_RATIO,
            MOI_KGM2,
            LENGTH_METERS,
            MIN_ANGLE.getRadians(),
            MAX_ANGLE.getRadians(),
            true,
            MIN_ANGLE.getRadians());

    private final ProfiledPIDController pid = new ProfiledPIDController(
            KP, KI, KD, new TrapezoidProfile.Constraints(CRUISE_VELOCITY_RPS, MAX_ACCELERATION_RPS2));
    private final ArmFeedforward ff = new ArmFeedforward(KS, KG, KV);

    private boolean isClosedLoop = false;
    private double voltageRequest = 0.0;
    private double appliedVolts = 0.0;

    public HoodIOSim() {
        pid.reset(Units.radiansToRotations(sim.getAngleRads()));
        SimState.getInstance().addCurrentDraw(sim::getCurrentDrawAmps, () -> appliedVolts);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        Rotation2d position = Rotation2d.fromRadians(sim.getAngleRads());
        if (isClosedLoop) {
            voltageRequest = pid.calculate(position.getRotations(), pid.getGoal().position)
                    + ff.calculate(
                            Units.rotationsToRadians(pid.getSetpoint().position),
                            Units.rotationsToRadians(pid.getSetpoint().velocity));
        }
        appliedVolts = SimulatedBattery.clamp(Volts.of(voltageRequest)).in(Volts);

        sim.setInputVoltage(appliedVolts);
        sim.update(Constants.LOOP_PERIOD_SECS);

        inputs.motorConnected = true;

        inputs.position = Rotation2d.fromRadians(sim.getAngleRads());
        inputs.velocityRPS = Units.radiansToRotations(sim.getVelocityRadPerSec());
        inputs.appliedVolts = appliedVolts;
        inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
        inputs.supplyCurrentAmps = sim.getCurrentDrawAmps()
                * appliedVolts
                / SimulatedBattery.getBatteryVoltage().in(Volts);
    }

    @Override
    public void setVoltage(double volts) {
        isClosedLoop = false;
        voltageRequest = volts;
    }

    @Override
    public void setSetpoint(Rotation2d position) {
        isClosedLoop = true;
        pid.setGoal(position.getRotations());
    }

    @Override
    public void configPID(double kP, double kI, double kD, double kS, double kV, double kG) {
        pid.setPID(kP, kI, kD);
        ff.setKs(kS);
        ff.setKv(kV);
        ff.setKg(kG);
    }
}
