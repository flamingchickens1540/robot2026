package org.team1540.robot2026.subsystems.shooter;

import static org.team1540.robot2026.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.SimState;
import org.team1540.robot2026.util.logging.LoggedTracer;
import org.team1540.robot2026.util.logging.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    private static boolean hasInstance = false;

    private final LoggedTunableNumber voltageKP = new LoggedTunableNumber("Shooter/Voltage/KP", VOLTAGE_KP);
    private final LoggedTunableNumber voltageKI = new LoggedTunableNumber("Shooter/Voltage/KI", VOLTAGE_KI);
    private final LoggedTunableNumber voltageKD = new LoggedTunableNumber("Shooter/Voltage/KD", VOLTAGE_KD);
    private final LoggedTunableNumber voltageKS = new LoggedTunableNumber("Shooter/Voltage/KS", VOLTAGE_KS);
    private final LoggedTunableNumber voltageKV = new LoggedTunableNumber("Shooter/Voltage/KV", VOLTAGE_KV);

    private final LoggedTunableNumber torqueKP = new LoggedTunableNumber("Shooter/Torque/KP", TORQUE_KP);
    private final LoggedTunableNumber torqueKI = new LoggedTunableNumber("Shooter/Torque/KI", TORQUE_KI);
    private final LoggedTunableNumber torqueKD = new LoggedTunableNumber("Shooter/Torque/KD", TORQUE_KD);
    private final LoggedTunableNumber torqueKS = new LoggedTunableNumber("Shooter/Torque/KS", TORQUE_KS);
    private final LoggedTunableNumber torqueKV = new LoggedTunableNumber("Shooter/Torque/KV", TORQUE_KV);

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final LinearFilter speedFilter = LinearFilter.movingAverage(5);

    @AutoLogOutput(key = "Shooter/FilteredRPM")
    private double filteredRPM = 0.0;

    @AutoLogOutput(key = "Shooter/SetpointRPM")
    private double setpointRPM;

    private final Alert leftDisconnected = new Alert("Shooter left motor disconnected", Alert.AlertType.kError);
    private final Alert rightDisconnected = new Alert("Shooter right motor disconnected", Alert.AlertType.kError);

    private Shooter(ShooterIO flyWheelsIO) {
        if (hasInstance) throw new IllegalStateException("Instance of shooter already exists");
        hasInstance = true;
        this.shooterIO = flyWheelsIO;
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        filteredRPM = speedFilter.calculate(getVelocityRPM());

        if (DriverStation.isDisabled()) {
            stop();
        }

        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            SimState.getInstance().addShooterData(inputs.leftVelocityRPM, inputs.rightVelocityRPM);
        }

        if (TORQUE_CONTROL) {
            LoggedTunableNumber.ifChanged(
                    hashCode(),
                    () -> shooterIO.configPID(
                            torqueKP.get(), torqueKI.get(), torqueKD.get(), torqueKS.get(), torqueKV.get()),
                    torqueKP,
                    torqueKI,
                    torqueKD,
                    torqueKS,
                    torqueKV);
        } else {
            LoggedTunableNumber.ifChanged(
                    hashCode(),
                    () -> shooterIO.configPID(
                            voltageKP.get(), voltageKI.get(), voltageKD.get(), voltageKS.get(), voltageKV.get()),
                    voltageKP,
                    voltageKI,
                    voltageKD,
                    voltageKS,
                    voltageKV);
        }

        leftDisconnected.set(!inputs.leftMotorConnected);
        rightDisconnected.set(!inputs.rightMotorConnected);

        // Log active command
        Command activeCmd = CommandScheduler.getInstance().requiring(this);
        Logger.recordOutput(
                "Shooter/ActiveCommand",
                activeCmd != null ? activeCmd.getName() + "_" + Integer.toHexString(activeCmd.hashCode()) : "None");

        LoggedTracer.record("Shooter");
    }

    public void runVelocity(double rpm) {
        setpointRPM = rpm;
        if (TORQUE_CONTROL) {
            shooterIO.runVelocityTorque(rpm);
        } else {
            shooterIO.runVelocityVoltage(rpm);
        }
    }

    public void setVoltage(double volts) {
        shooterIO.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    public void stop() {
        setVoltage(0);
    }

    public double getVelocityRPM() {
        return (inputs.leftVelocityRPM + inputs.rightVelocityRPM) / 2.0;
    }

    @AutoLogOutput(key = "Shooter/AtSetpoint")
    public boolean atSetpoint() {
        return MathUtil.isNear(setpointRPM, filteredRPM, ERROR_TOLERANCE_RPM);
    }

    public Command commandVelocity(DoubleSupplier setpoint) {
        return runEnd(() -> runVelocity(setpoint.getAsDouble()), this::stop).withName("ShooterVelocityCommand");
    }

    public Command stopCommand() {
        return runOnce(this::stop).withName("ShooterStopCommand");
    }

    public static Shooter createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using a real shooter on sim robot", false);
        }
        return new Shooter(new ShooterIOTalonFX());
    }

    public static Shooter createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using a sim shooter on real robot", false);
        }
        return new Shooter(new ShooterIOSim());
    }

    public static Shooter createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("using a dummy shooter on real robot", false);
        }
        return new Shooter(new ShooterIO() {});
    }
}
