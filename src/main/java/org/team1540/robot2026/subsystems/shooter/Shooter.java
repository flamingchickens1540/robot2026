package org.team1540.robot2026.subsystems.shooter;

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
import org.team1540.robot2026.util.LoggedTracer;
import org.team1540.robot2026.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    private static boolean hasInstance = false;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", ShooterConstants.KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", ShooterConstants.KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", ShooterConstants.KD);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", ShooterConstants.KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", ShooterConstants.KV);

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final LinearFilter speedFilter = LinearFilter.movingAverage(5);

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

        Logger.recordOutput("Shooter/FilteredSpeedRPM", speedFilter.calculate(getVelocityRPM()));

        if (DriverStation.isDisabled()) {
            stop();
        }

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> shooterIO.configPID(kP.get(), kI.get(), kD.get(), kV.get(), kS.get()),
                kP,
                kI,
                kD,
                kS,
                kV);

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
        speedFilter.reset();
        shooterIO.setSpeed(rpm);
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
        return MathUtil.isNear(
                setpointRPM, speedFilter.calculate(getVelocityRPM()), ShooterConstants.ERROR_TOLERANCE_RPM);
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
