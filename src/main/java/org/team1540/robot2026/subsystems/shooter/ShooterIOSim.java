package org.team1540.robot2026.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static org.team1540.robot2026.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.SimState;

public class ShooterIOSim implements ShooterIO {
    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), MOI_KGM2, GEAR_RATIO),
            DCMotor.getKrakenX60Foc(2));

    private final PIDController pid = new PIDController(KP, KI, KD);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(KS, KV);

    private double voltageRequest = 0.0;
    private double appliedVolts = 0.0;
    private boolean isClosedLoop = false;

    public ShooterIOSim() {
        pid.reset();
        SimState.getInstance().addCurrentDraw(sim::getCurrentDrawAmps, () -> appliedVolts);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        double velocityRPS = sim.getAngularVelocityRPM() / 60.0;
        if (isClosedLoop) {
            voltageRequest = pid.calculate(velocityRPS, pid.getSetpoint()) + ff.calculate(pid.getSetpoint());
        }
        appliedVolts = SimulatedBattery.clamp(Volts.of(voltageRequest)).in(Volts);

        sim.setInputVoltage(appliedVolts);
        sim.update(Constants.LOOP_PERIOD_SECS);

        inputs.leftMotorConnected = true;
        inputs.leftVelocityRPM = sim.getAngularVelocityRPM();
        inputs.leftAppliedVolts = appliedVolts;
        inputs.leftStatorCurrentAmps = sim.getCurrentDrawAmps();
        inputs.leftSupplyCurrentAmps = sim.getCurrentDrawAmps()
                * appliedVolts
                / SimulatedBattery.getBatteryVoltage().in(Volts);

        inputs.rightMotorConnected = true;
        inputs.rightVelocityRPM = sim.getAngularVelocityRPM();
        inputs.rightAppliedVolts = appliedVolts;
        inputs.rightStatorCurrentAmps = sim.getCurrentDrawAmps();
        inputs.rightSupplyCurrentAmps = sim.getCurrentDrawAmps()
                * appliedVolts
                / SimulatedBattery.getBatteryVoltage().in(Volts);
    }

    @Override
    public void setVoltage(double volts) {
        voltageRequest = volts;
        isClosedLoop = false;
    }

    @Override
    public void setSpeed(double rpm) {
        pid.setSetpoint(rpm / 60.0);
        isClosedLoop = true;
    }

    @Override
    public void configPID(double kP, double kI, double kD, double kS, double kV) {
        pid.setPID(kP, kI, kD);
        ff.setKs(kS);
        ff.setKv(kV);
    }
}
