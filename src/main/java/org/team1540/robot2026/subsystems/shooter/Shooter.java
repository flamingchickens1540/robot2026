package org.team1540.robot2026.subsystems.shooter;

import static org.team1540.robot2026.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.util.LoggedTracer;
import org.team1540.robot2026.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    private static boolean hasInstance = false;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP");
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI");
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD");
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS");
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV");

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double setpointRPM = 0.0;
    private final LinearFilter velocityFilter = LinearFilter.movingAverage(5);

    private final Alert leaderDisconnectedAlert = new Alert("Shooter leader disconnected", Alert.AlertType.kError);
    private final Alert followerDisconnectedAlert = new Alert("Shooter follower disconnected", Alert.AlertType.kError);

    private Shooter(ShooterIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of shooter already exists");
        hasInstance = true;
        this.io = io;
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput(
                "Shooter/FilteredVelocity",
                velocityFilter.calculate((inputs.velocityRPM[0] + inputs.velocityRPM[1]) / 2.0));

        if (DriverStation.isDisabled()) stop();

        LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.configPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get()), kP, kI, kD, kS, kV);

        leaderDisconnectedAlert.set(!inputs.leaderConnected);
        followerDisconnectedAlert.set(!inputs.followerConnected);

        LoggedTracer.record("Shooter");
    }

    public void runVelocity(double velocityRPM) {
        if (velocityRPM == 0) stop();
        else io.runVelocity(velocityRPM);

        setpointRPM = velocityRPM;
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        setVoltage(0);
    }

    public double getVelocityRPM() {
        return (inputs.velocityRPM[0] + inputs.velocityRPM[1]) / 2.0;
    }

    @AutoLogOutput(key = "Shooter/SetpointRPM")
    public double getSetpointRPM() {
        return setpointRPM;
    }

    @AutoLogOutput(key = "Shooter/AtSetpoint")
    public boolean isAtSetpoint() {
        return MathUtil.isNear(velocityFilter.lastValue(), setpointRPM, VELOCITY_ERR_TOLERANCE_RPM);
    }

    public Command runVelocityCommand(DoubleSupplier velocityRPM) {
        return runEnd(() -> runVelocity(velocityRPM.getAsDouble()), this::stop);
    }

    public static Shooter createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real shooter on simulated robot", false);
        }
        return new Shooter(new ShooterIOTalonFX());
    }

    public static Shooter createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated shooter on real robot", false);
        }
        return new Shooter(new ShooterIOSim());
    }

    public static Shooter createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy shooter on real robot", false);
        }
        return new Shooter(new ShooterIO() {});
    }
}
