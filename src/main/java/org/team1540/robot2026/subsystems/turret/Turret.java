package org.team1540.robot2026.subsystems.turret;

import static org.team1540.robot2026.subsystems.turret.TurretConstants.*;

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

public class Turret extends SubsystemBase {
    private static boolean hasInstance = false;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", KD);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", KV);

    private final Alert motorDisconnectedAlert = new Alert("Turret motor disconnected", Alert.AlertType.kError);

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private Rotation2d setpoint = Rotation2d.kZero;

    private Turret(TurretIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of elevator already exists");
        hasInstance = true;
        this.io = io;
        resetPosition(MIN_ANGLE);
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        if (DriverStation.isDisabled()) stop();

        LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.configPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get()), kP, kI, kD, kS, kV);

        motorDisconnectedAlert.set(!inputs.motorConnected);

        LoggedTracer.record("Turret");
    }

    public void stop() {
        io.setVoltage(0);
    }

    @AutoLogOutput(key = "Turret/AtSetpoint")
    public boolean isAtSetpoint() {
        return MathUtil.isNear(getSetpoint().getDegrees(), getPosition().getDegrees(), POS_ERR_TOLERANCE_DEGREES);
    }

    @AutoLogOutput(key = "Turret/Setpoint")
    public Rotation2d getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(Rotation2d position) {
        position = Rotation2d.fromDegrees(
                MathUtil.clamp(position.getDegrees(), MIN_ANGLE.getDegrees(), MAX_ANGLE.getDegrees()));
        setpoint = position;
        io.setSetpoint(position);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void resetPosition(Rotation2d position) {
        io.resetPosition(Rotation2d.fromDegrees(
                MathUtil.clamp(position.getDegrees(), MIN_ANGLE.getDegrees(), MAX_ANGLE.getDegrees())));
    }

    public Rotation2d getPosition() {
        return inputs.position;
    }

    public double getVelocityRPS() {
        return inputs.velocityRPS;
    }

    public void setBrakeMode(boolean isBrakeMode) {
        io.setBrakeMode(isBrakeMode);
    }

    public Command commandToSetpoint(Supplier<Rotation2d> position) {
        return runEnd(() -> setSetpoint(position.get()), this::stop).withName("TurretSetpointCommand");
    }

    public static Turret createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real turret on simulated robot", false);
        }
        return new Turret(new TurretIOTalonFX());
    }

    public static Turret createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated turret on real robot", false);
        }
        return new Turret(new TurretIOSim());
    }

    public static Turret createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy turret on real robot", false);
        }
        return new Turret(new TurretIO() {});
    }
}
