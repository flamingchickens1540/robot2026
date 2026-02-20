package org.team1540.robot2026.subsystems.climber;

import static org.team1540.robot2026.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.util.LoggedTracer;
import org.team1540.robot2026.util.LoggedTunableNumber;

public class Climber extends SubsystemBase {
    private boolean hasInstance = false;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Climber/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Climber/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Climber/kD", KD);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Climber/kS", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Climber/kV", KV);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Climber/kG", KG);

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private double setpointMeters;

    private Climber(ClimberIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of climber already exists");
        hasInstance = true;
        this.io = io;
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        if (RobotState.isDisabled()) stop();

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configFF(kS.get(), kV.get(), kG.get()), kS, kV, kG);

        LoggedTracer.record("Climber");
    }

    public void setPosition(double positionMeters) {
        positionMeters = MathUtil.clamp(positionMeters, 0.0, MAX_HEIGHT_M);
        setpointMeters = positionMeters;
        io.setSetpoint(setpointMeters);
    }

    @AutoLogOutput(key = "Climber/AtSetpoint")
    public boolean isAtSetpoint() {
        return (MathUtil.isNear(setpointMeters, inputs.leftMotorPositionMeters, POS_ERR_TOLERANCE_M)
                        && MathUtil.isNear(setpointMeters, inputs.rightMotorPositionMeters, POS_ERR_TOLERANCE_M))
                || (inputs.atLowerLimit && setpointMeters <= 0);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stop() {
        io.setVoltage(0.0);
    }

    @AutoLogOutput(key = "Climber/Setpoint")
    public double getSetpoint() {
        return setpointMeters;
    }

    public double[] getPosition() {
        return new double[] {inputs.leftMotorPositionMeters, inputs.rightMotorPositionMeters};
    }

    public double[] getVelocity() {
        return new double[] {inputs.leftMotorVelocityMPS, inputs.rightMotorVelocityMPS};
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
        io.setSetpoint(inputs.leftMotorPositionMeters);
    }

    public Command climbCommand(double positionMeters) {
        return Commands.runEnd(
                () -> {
                    io.setSetpoint(positionMeters);
                },
                () -> {
                    io.setVoltage(0);
                },
                this);
    }

    public static Climber createReal() {
        return new Climber(new ClimberIOReal());
    }

    public static Climber createSim() {
        return new Climber(new ClimberIOSim());
    }

    public static Climber createDummy() {
        return new Climber(new ClimberIO() {});
    }
}
