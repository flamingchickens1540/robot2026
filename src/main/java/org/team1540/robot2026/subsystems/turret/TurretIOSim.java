package org.team1540.robot2026.subsystems.turret;

import static edu.wpi.first.units.Units.Volts;
import static org.team1540.robot2026.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.SimState;

public class TurretIOSim implements TurretIO {
    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), GEAR_RATIO, MOI_KGM2), DCMotor.getKrakenX44(1));

    private final ProfiledPIDController pid = new ProfiledPIDController(
            KP, KI, KD, new TrapezoidProfile.Constraints(CRUISE_VELOCITY_RPS, MAX_ACCEL_RPS2));
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(KS, KV);

    private boolean isClosedLoop = false;
    private double voltageRequest = 0.0;
    private double appliedVolts = 0.0;

    public TurretIOSim() {
        pid.reset(sim.getAngularPositionRotations());
        SimState.getInstance().addCurrentDraw(sim::getCurrentDrawAmps, () -> appliedVolts);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        Rotation2d position = Rotation2d.fromRotations(sim.getAngularPositionRotations());
        if (isClosedLoop) {
            voltageRequest = pid.calculate(position.getRotations(), pid.getGoal().position)
                    + ff.calculate(pid.getSetpoint().velocity);
        }
        appliedVolts = SimulatedBattery.clamp(Volts.of(voltageRequest)).in(Volts);

        sim.setInputVoltage(appliedVolts);
        sim.update(Constants.LOOP_PERIOD_SECS);

        inputs.connected = true;
        inputs.position = Rotation2d.fromRotations(sim.getAngularPositionRotations());
        inputs.positionTimestamp = Timer.getTimestamp();
        inputs.velocityRPS = sim.getAngularVelocityRPM() / 60.0;
        inputs.appliedVolts = appliedVolts;
        inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
        inputs.supplyCurrentAmps = sim.getCurrentDrawAmps()
                * appliedVolts
                / SimulatedBattery.getBatteryVoltage().in(Volts);

        inputs.bigEncoderConnected = true;
        inputs.bigEncoderPosition = inputs.position.times((double) MAIN_GEAR_TEETH / BIG_ENCODER_TEETH);
        inputs.smallEncoderConnected = true;

        inputs.smallEncoderPosition = inputs.position.times((double) MAIN_GEAR_TEETH / SMALL_ENCODER_TEETH);
    }

    @Override
    public void setVoltage(double volts) {
        isClosedLoop = false;
        voltageRequest = volts;
    }

    @Override
    public void setSetpoint(Rotation2d rotation, double voltageFF) {
        isClosedLoop = true;
        pid.setGoal(new TrapezoidProfile.State(rotation.getRotations(), voltageFF / ff.getKv()));
    }

    @Override
    public void configPID(double kP, double kI, double kD) {
        pid.setPID(kP, kI, kD);
    }

    @Override
    public void configFF(double kS, double kV) {
        ff.setKs(kS);
        ff.setKv(kV);
    }
}
