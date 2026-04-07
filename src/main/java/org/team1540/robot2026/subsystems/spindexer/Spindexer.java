package org.team1540.robot2026.subsystems.spindexer;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.util.LoggedTracer;

public class Spindexer extends SubsystemBase {
    private static boolean hasInstance = false;

    private final SpindexerIO io;
    private final SpindexerSensorIO sensorIO;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
    private final SpindexerSensorIOInputsAutoLogged sensorInputs = new SpindexerSensorIOInputsAutoLogged();

    private final Alert spinMotorDisconnectedAlert = new Alert("Spindexer motor disconnected", Alert.AlertType.kError);
    private final Alert feederMotorDisconnectedAlert = new Alert("Feeder motor disconnected", Alert.AlertType.kError);

    private int numBallsCounted = 0;
    private double lastLaserCanMeasurementMM = 0;
    private final LinkedList<Double> laserCanMeasurementHistory = new LinkedList<>();
    private final LinearFilter bpsFilter = LinearFilter.movingAverage(3);

    private Spindexer(SpindexerIO io, SpindexerSensorIO sensorIO) {
        if (hasInstance) throw new IllegalStateException("Instance of spindexer already exists");
        hasInstance = true;
        this.io = io;
        this.sensorIO = sensorIO;
    }

    private void calculateBPS(double threshold) {
        int isBall = 0;
        if (lastLaserCanMeasurementMM != 0
                && sensorInputs.distanceMM <=threshold) { // could have issues skipping balls if ball goes fully through without getting counted
            numBallsCounted++;
            laserCanMeasurementHistory.addLast(Timer.getTimestamp());
            isBall = 1;
        }
        if (!laserCanMeasurementHistory.isEmpty()){
            if (laserCanMeasurementHistory.getFirst() < System.currentTimeMillis() - 3 * 1000) {
                laserCanMeasurementHistory.removeFirst();
            }
        }
        lastLaserCanMeasurementMM = sensorInputs.distanceMM;
        Logger.recordOutput("RealOutputs/Spindexer/balls", numBallsCounted);
        Logger.recordOutput("RealOutputs/Spindexer/bps3/filter", bpsFilter.calculate(isBall));
        Logger.recordOutput("RealOutputs/Spindexer/bps3/list", laserCanMeasurementHistory.size()/3);
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        sensorIO.updateInputs(sensorInputs);
        io.updateInputs(inputs);
        calculateBPS(1);

        Logger.processInputs("Spindexer", inputs);
        Logger.processInputs("Spindexer/Sensor", sensorInputs);

        if (DriverStation.isDisabled()) stop();

        spinMotorDisconnectedAlert.set(!inputs.spinMotorConnected);
        feederMotorDisconnectedAlert.set(!inputs.feederMotorConnected);

        Command activeCmd = CommandScheduler.getInstance().requiring(this);
        Logger.recordOutput(
                "Spindexer/ActiveCommand",
                activeCmd != null ? activeCmd.getName() + "_" + Integer.toHexString(activeCmd.hashCode()) : "None");

        LoggedTracer.record("Spindexer");
    }

    public void setMotorSpeeds(double spinPercent, double feederPercent) {
        io.setMotorVoltages(spinPercent * 12.0, feederPercent * 12.0);
    }

    public void stop() {
        setMotorSpeeds(0.0, 0.0);
    }

    public Command runCommand(DoubleSupplier spinPercent, DoubleSupplier feederPercent) {
        return runEnd(() -> setMotorSpeeds(spinPercent.getAsDouble(), feederPercent.getAsDouble()), this::stop)
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
            DriverStation.reportWarning("Using simulated turret on real robot", false);
        }
        return new Spindexer(new SpindexerIOSim(), new SpindexerSensorIO() {});
    }
}
