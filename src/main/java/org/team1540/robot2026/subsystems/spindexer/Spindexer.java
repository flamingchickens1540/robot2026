package org.team1540.robot2026.subsystems.spindexer;

import static org.team1540.robot2026.subsystems.spindexer.SpindexerConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.SimState;
import org.team1540.robot2026.util.logging.LoggedTracer;

public class Spindexer extends SubsystemBase {
    private static boolean hasInstance = false;

    private final SpindexerIO io;
    private final SpindexerSensorIO sensorIO;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
    private final SpindexerSensorIOInputsAutoLogged sensorInputs = new SpindexerSensorIOInputsAutoLogged();

    private final Alert spinMotorDisconnectedAlert = new Alert("Spindexer motor disconnected", Alert.AlertType.kError);
    private final Alert feederMotorDisconnectedAlert = new Alert("Feeder motor disconnected", Alert.AlertType.kError);
    private final Alert feederMotor2DisconnectedAlert =
            new Alert("Hopper feeder motor disconnected", Alert.AlertType.kError);

    private int numBallsCounted = 0;
    private double lastSensorMeasurementMM;
    private double lastBallTimestamp = Double.MAX_VALUE;
    private final LinearFilter bpsFilter = LinearFilter.movingAverage(120);

    private Spindexer(SpindexerIO io, SpindexerSensorIO sensorIO) {
        if (hasInstance) throw new IllegalStateException("Instance of spindexer already exists");
        hasInstance = true;
        this.io = io;
        this.sensorIO = sensorIO;
    }

    private void calculateBPS() {
        if (!(lastSensorMeasurementMM <= SENSOR_THRESHOLD_MM) && sensorInputs.distanceMM <= SENSOR_THRESHOLD_MM) {
            double currentTime = Timer.getTimestamp();
            if (currentTime - lastBallTimestamp != 0) { // Ensure no division by zero
                bpsFilter.calculate(1 / (currentTime - lastBallTimestamp));
            }
            numBallsCounted++;
            lastBallTimestamp = Timer.getTimestamp();
        }
        lastSensorMeasurementMM = sensorInputs.distanceMM;
        Logger.recordOutput("RealOutputs/Spindexer/BallCounter/NumBalls", numBallsCounted);
        Logger.recordOutput("RealOutputs/Spindexer/BallCounter/BPS", bpsFilter.lastValue());
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        sensorIO.updateInputs(sensorInputs);
        io.updateInputs(inputs);
        calculateBPS();

        Logger.processInputs("Spindexer", inputs);
        Logger.processInputs("Spindexer/Sensor", sensorInputs);

        if (DriverStation.isDisabled()) stop();

        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            SimState.getInstance()
                    .addSpindexerData(inputs.spinAppliedVolts, inputs.feeder1AppliedVolts, inputs.feeder2AppliedVolts);
        }

        spinMotorDisconnectedAlert.set(!inputs.spinMotorConnected);
        feederMotorDisconnectedAlert.set(!inputs.feederMotorConnected);
        feederMotor2DisconnectedAlert.set(!inputs.feeder2MotorConnected);

        Command activeCmd = CommandScheduler.getInstance().requiring(this);
        Logger.recordOutput(
                "Spindexer/ActiveCommand",
                activeCmd != null ? activeCmd.getName() + "_" + Integer.toHexString(activeCmd.hashCode()) : "None");

        LoggedTracer.record("Spindexer");
    }

    public void setMotorSpeeds(double spinPercent, double feederPercent, double feeder2Percent) {
        io.setMotorVoltages(spinPercent * 12.0, feederPercent * 12.0, feeder2Percent * 12.0);
    }

    public void stop() {
        setMotorSpeeds(0.0, 0.0, 0.0);
    }

    public Command runCommand(DoubleSupplier spinPercent, DoubleSupplier feederPercent, DoubleSupplier feeder2Percent) {
        return runEnd(
                        () -> setMotorSpeeds(
                                spinPercent.getAsDouble(), feederPercent.getAsDouble(), feeder2Percent.getAsDouble()),
                        this::stop)
                .withName("SpindexerRunCommand");
    }

    public static Spindexer createReal() {
        return new Spindexer(new SpindexerIOTalonFX(), new SpindexerSensorIOReal());
    }

    public static Spindexer createDummy() {
        return new Spindexer(new SpindexerIO() {}, new SpindexerSensorIO() {});
    }

    public static Spindexer createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated spindexer on real robot", false);
        }
        return new Spindexer(new SpindexerIOSim(), new SpindexerSensorIO() {});
    }
}
