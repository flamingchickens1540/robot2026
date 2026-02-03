package org.team1540.robot2026.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team1540.robot2026.subsystems.climber.ClimberConstants.*;

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
    private double setPointMeters;

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
        setPointMeters = positionMeters;
        io.setSetPoint(setPointMeters);
    }

    @AutoLogOutput(key = "Climber/AtSetpoint")
    public boolean isAtSetpoint() {
        return (MathUtil.isNear(setPointMeters, inputs.leftMotorPosition, POS_ERR_TOLERANCE_M) && MathUtil.isNear(setPointMeters, inputs.rightMotorPosition, POS_ERR_TOLERANCE_M))
                || (inputs.atLowerLimit && setPointMeters <= 0);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stop() {
        io.setVoltage(0.0);
    }

    @AutoLogOutput(key = "Climber/Setpoint")
    public double getSetpoint() {
        return setPointMeters;
    }

    public double[] getPosition() {
        return new double[] {inputs.leftMotorPosition, inputs.rightMotorPosition};
    }

    public double[] getVelocity() {
        return new double[] {inputs.leftMotorVelocityRPM, inputs.rightMotorVelocityRPM};
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
        io.setSetPoint(inputs.leftMotorPosition);
    }

    public static Climber createReal() {
        return new Climber(new ClimberIOReal());
    }

    public static Climber createDummy() {
        return new Climber(new ClimberIO() {});
    }
}
