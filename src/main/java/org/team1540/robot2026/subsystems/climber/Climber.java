package org.team1540.robot2026.subsystems.climber;

import static org.team1540.robot2026.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.util.LoggedTracer;
import org.team1540.robot2026.util.LoggedTunableNumber;

public class Climber extends SubsystemBase {
    private static boolean hasInstance = false;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Climber/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Climber/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Climber/kD", KD);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Climber/kS", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Climber/kV", KV);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Climber/kG", KG);

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private double setpointMeters;

    private final Alert motorDisconnected = new Alert("Climber motor disconnected", Alert.AlertType.kError);

    private Climber(ClimberIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of climber already exists");
        hasInstance = true;
        this.io = io;
        hasInstance = true;
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        if (RobotState.isDisabled()) stop();

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configFF(kS.get(), kV.get(), kG.get()), kS, kV, kG);

        motorDisconnected.set(!inputs.motorConnected);

        LoggedTracer.record("Climber");
    }

    public void setPosition(double positionMeters) {
        positionMeters = MathUtil.clamp(positionMeters, 0.0, MAX_HEIGHT_M);
        setpointMeters = positionMeters;
        io.setSetpoint(setpointMeters);
    }

    @AutoLogOutput(key = "Climber/AtSetpoint")
    public boolean atSetpoint() {
        return MathUtil.isNear(setpointMeters, inputs.positionMeters, POS_ERR_TOLERANCE_M)
                || (inputs.atLowerLimit && setpointMeters <= 0);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stop() {
        io.setVoltage(0.0);
    }

    @AutoLogOutput(key = "Climber/Setpoint")
    public double getSetpointMeters() {
        return setpointMeters;
    }

    public double getPositionMeters() {
        return inputs.positionMeters;
    }

    public double getVelocityMPS() {
        return inputs.velocityMPS;
    }

    public void setBrakeMode(boolean isBrakeMode) {
        io.setBrakeMode(isBrakeMode);
    }

    public boolean getUpperLimit() {
        return inputs.atUpperLimit;
    }

    public boolean getLowerLimit() {
        return inputs.atLowerLimit;
    }

    public void holdPosition() {
        io.setSetpoint(inputs.positionMeters);
    }

    public Command setpointCommand(double positionMeters) {
        return Commands.runEnd(() -> setPosition(positionMeters), this::stop, this);
    }

    public static Climber createReal() {
        return new Climber(new ClimberIOTalonFX());
    }

    public static Climber createSim() {
        return new Climber(new ClimberIOSim());
    }

    public static Climber createDummy() {
        return new Climber(new ClimberIO() {});
    }
}
