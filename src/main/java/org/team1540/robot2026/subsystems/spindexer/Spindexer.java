package org.team1540.robot2026.subsystems.spindexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;

import java.util.function.BooleanSupplier;

import static org.team1540.robot2026.subsystems.spindexer.SpindexerConstants.VELOCITY_ERR_TOLERANCE_RPM;

public class Spindexer extends SubsystemBase {
    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    private double feederSetpointRPM = 0.0;

    private static boolean hasInstance = false;

    private Spindexer(SpindexerIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of spindexer already exists");
        hasInstance = true;
        this.io = io;
    }

    public static Spindexer createReal() {
        if (Constants.CURRENT_MODE.equals(Constants.Mode.REAL)) {
            DriverStation.reportWarning("Using real indexer on simulated robot", false);
        }
        return new Spindexer(new SpindexerIOTalonFX());
    }

    public static Spindexer createSim() {
        if (Constants.CURRENT_MODE.equals(Constants.Mode.REAL)) {
            DriverStation.reportWarning("Using simulated spindexer on real robot", false);
        }
        return new Spindexer(new SpindexerIOSim());
    }

    public static Spindexer createDummy() {
        if (Constants.CURRENT_MODE.equals(Constants.Mode.REAL)) {
            DriverStation.reportWarning("Using dummy spindexer on real robot", false);
        }
        return new Spindexer(new SpindexerIO() {
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);

        if (RobotState.isDisabled()){
            stopAll();
        }
    }

    public void setIntakePercent(double percent) {
        io.setIntakeVoltage(percent * 12.0);
    }

    public void setFeederVelocity(double setpointRPM) {
        feederSetpointRPM = setpointRPM;
        io.setFeederVelocity(setpointRPM);
    }

    public void setFeederPercent(double percent) {
        io.setFeederVoltage(12.0 * percent);
    }

    public boolean isFeederAtSetpoint() {
        return Math.abs(getFeederVelocityError()) < VELOCITY_ERR_TOLERANCE_RPM;
    }

    public void stopFeeder() {
        io.setFeederVoltage(0);
    }

    public void stopIntake() {
        io.setIntakeVoltage(0);
    }

    public void stopAll() {
        io.setIntakeVoltage(0);
        io.setFeederVoltage(0);
    }

    @AutoLogOutput(key = "Intake/Feeder/setpointRPM")
    public double getFeederVelocitySetpoint() {
        return feederSetpointRPM;
    }

    @AutoLogOutput(key = "Intake/Feeder/velocityErrorRPM")
    public double getFeederVelocityError() {
        return inputs.feederVelocityRPM - getFeederVelocitySetpoint();
    }

    public Command feedToAmp() {
        return Commands.runOnce(() -> io.setFeederVelocity(-600), this);
    }

    public Command feedToShooter() {
        return Commands.runOnce(() -> io.setFeederVelocity(1200), this);
    }

    public Command commandRunIntake(double percent) {
        return Commands.startEnd(
                () -> this.setIntakePercent(percent),
                () -> this.setIntakePercent(0),
                this
        );
    }

    public void setIntakeBrakeMode(boolean isBrake) {
        io.setIntakeBrakeMode(isBrake);
    }

    public double getIntakeVoltage() {
        return inputs.intakeVoltage;
    }

    public double getIntakeCurrent() {
        return inputs.intakeCurrentAmps;
    }

}