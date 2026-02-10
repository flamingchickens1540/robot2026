package org.team1540.robot2026.subsystems.hood;

import static org.team1540.robot2026.subsystems.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.util.LoggedTracer;
import org.team1540.robot2026.util.LoggedTunableNumber;

public class Hood extends SubsystemBase {
    private static boolean hasInstance = false;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", 0.0);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Hood/kI", 0.0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", 0.0);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS", 0.0);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV", 0.0);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/kG", 0.0);

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private Rotation2d setpoint = Rotation2d.kZero;

    private final Alert motorDisconnectedAlert = new Alert("Hood motor disconnected", Alert.AlertType.kError);

    private Hood(HoodIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of hood already exists");
        hasInstance = true;
        this.io = io;
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
        setpoint = position;
        io.setSetpoint(position);
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
        return runEnd(() -> setSetpoint(position.get()), this::stop);
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
