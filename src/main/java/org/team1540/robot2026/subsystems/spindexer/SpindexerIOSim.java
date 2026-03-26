package org.team1540.robot2026.subsystems.spindexer;

import static edu.wpi.first.units.Units.Volts;
import static org.team1540.robot2026.subsystems.spindexer.SpindexerConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.SimState;

public class SpindexerIOSim implements SpindexerIO {
    private final DCMotorSim spinSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, SPIN_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));
    private final DCMotorSim feederSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, FEEDER_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));

    private double spinRequestedVolts = 0.0;
    private double spinAppliedVolts = 0.0;

    private double feederRequestedVolts = 0.0;
    private double feederAppliedVolts = 0.0;

    public SpindexerIOSim() {
        SimState.getInstance().addCurrentDraw(spinSim::getCurrentDrawAmps, () -> spinAppliedVolts);
        SimState.getInstance().addCurrentDraw(feederSim::getCurrentDrawAmps, () -> feederAppliedVolts);
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        spinAppliedVolts = SimulatedBattery.clamp(Volts.of(spinRequestedVolts)).in(Volts);
        feederAppliedVolts =
                SimulatedBattery.clamp(Volts.of(feederRequestedVolts)).in(Volts);

        spinSim.setInputVoltage(spinAppliedVolts);
        feederSim.setInputVoltage(feederAppliedVolts);
        spinSim.update(Constants.LOOP_PERIOD_SECS);
        feederSim.update(Constants.LOOP_PERIOD_SECS);

        inputs.spinMotorConnected = true;
        inputs.spinVelocityRPS = spinSim.getAngularVelocityRPM() / 60.0;
        inputs.spinAppliedVolts = spinAppliedVolts;
        inputs.spinStatorCurrentAmps = spinSim.getCurrentDrawAmps();
        inputs.spinSupplyCurrentAmps = spinSim.getCurrentDrawAmps()
                * spinAppliedVolts
                / SimulatedBattery.getBatteryVoltage().in(Volts);

        inputs.feederMotorConnected = true;
        inputs.feederVelocityRPS = feederSim.getAngularVelocityRPM() / 60.0;
        inputs.feederAppliedVolts = feederAppliedVolts;
        inputs.feederStatorCurrentAmps = feederSim.getCurrentDrawAmps();
        inputs.feederSupplyCurrentAmps = feederSim.getCurrentDrawAmps()
                * feederAppliedVolts
                / SimulatedBattery.getBatteryVoltage().in(Volts);
    }

    @Override
    public void setMotorVoltages(double spinVolts, double feederVolts) {
        spinRequestedVolts = spinVolts;
        feederRequestedVolts = feederVolts;
    }
}
