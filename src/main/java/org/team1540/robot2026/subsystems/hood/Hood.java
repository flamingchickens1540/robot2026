package org.team1540.robot2026.subsystems.hood;

import static org.team1540.robot2026.subsystems.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.util.LoggedTracer;
import org.team1540.robot2026.util.LoggedTunableNumber;

public class Hood extends SubsystemBase {
    private static boolean hasInstance = false;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Hood/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", KD);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV", KV);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/kG", KG);

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private Rotation2d setpoint = Rotation2d.kZero;

    private final Alert motorDisconnectedAlert = new Alert("Hood motor disconnected", Alert.AlertType.kError);

    private Hood(HoodIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of hood already exists");
        hasInstance = true;
        this.io = io;
        resetPosition(MIN_ANGLE);
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        if (DriverStation.isDisabled()) stop();

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.configPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kG.get()),
                kP,
                kI,
                kD,
                kS,
                kV,
                kG);

        motorDisconnectedAlert.set(!inputs.motorConnected);

        LoggedTracer.record("Hood");
    }

    public void setSetpoint(Rotation2d position) {
        setpoint = Rotation2d.fromDegrees(
                MathUtil.clamp(position.getDegrees(), MIN_ANGLE.getDegrees(), MAX_ANGLE.getDegrees()));
        io.setSetpoint(setpoint);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        setVoltage(0.0);
    }

    public Rotation2d getPosition() {
        return inputs.position;
    }

    public void resetPosition(Rotation2d position) {
        io.resetPosition(Rotation2d.fromDegrees(
                MathUtil.clamp(position.getDegrees(), MIN_ANGLE.getDegrees(), MAX_ANGLE.getDegrees())));
    }

    public double getVelocityRPS() {
        return inputs.velocityRPS;
    }

    @AutoLogOutput(key = "Hood/Setpoint")
    public Rotation2d getSetpoint() {
        return setpoint;
    }

    @AutoLogOutput(key = "Hood/AtSetpoint")
    public boolean isAtSetpoint() {
        return MathUtil.isNear(
                getPosition().getDegrees(), getSetpoint().getDegrees(), POSITION_ERR_TOLERANCE.getDegrees());
    }

    public Command setpointCommand(Supplier<Rotation2d> position) {
        return runEnd(() -> setSetpoint(position.get()), this::stop).withName("HoodSetpointCommands");
    }

    public Command zeroCommand() {
        return runOnce(() -> setVoltage(-0.1))
                .andThen(
                        Commands.waitUntil(
                                new Trigger(() -> inputs.statorCurrentAmps >= ZERO_CURRENT_AMPS).debounce(0.5)),
                        runOnce(() -> resetPosition(MIN_ANGLE)));
    }

    public static Hood createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL)
            DriverStation.reportWarning("Using real hood on simulated robot", false);
        return new Hood(new HoodIOTalonFX());
    }

    public static Hood createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL)
            DriverStation.reportWarning("Using simulated hood on real robot", false);
        return new Hood(new HoodIO() {});
    }

    public static Hood createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy hood on real robot", false);
        return new Hood(new HoodIO() {});
    }
}
