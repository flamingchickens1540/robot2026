package org.team1540.robot2026.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    private FlywheelsIO flywheelsIO;
    // TODO
    private final LinearFilter leftSpeedFilter = LinearFilter.movingAverage(20);
    private final LinearFilter rightSpeedFilter = LinearFilter.movingAverage(20);

    private double leftFlywheelSetpointRPM;
    private double rightFlywheelSetpointRPM;
    private final FlywheelsIOInputsAutoLogged flywheelInputs = new FlywheelsIOInputsAutoLogged();

    private final LoggedTunableNumber flywheelsKP =
            new LoggedTunableNumber("Shooter/Flywheels/kP", ShooterConstants.Flywheels.KP);
    private final LoggedTunableNumber flywheelsKI =
            new LoggedTunableNumber("Shooter/Flywheels/kI", ShooterConstants.Flywheels.KI);
    private final LoggedTunableNumber flywheelsKD =
            new LoggedTunableNumber("Shooter/Flywheels/kD", ShooterConstants.Flywheels.KD);
    private final LoggedTunableNumber flywheelsKV =
            new LoggedTunableNumber("Shooter/Flywheels/kV", ShooterConstants.Flywheels.KV);
    private final LoggedTunableNumber flywheelsKS =
            new LoggedTunableNumber("Shooter/Flywheels/KS", ShooterConstants.Flywheels.KS);
    // TODO: public final ShooterLerp lerp = new ShooterLerp().put();
    // TODO: add motor connected thingy

    private static boolean hasInstance = false;

    private Shooter(FlywheelsIO flyWheelsIO) {
        if (hasInstance) throw new IllegalStateException("Instance of shooter already exists");
        hasInstance = true;
        this.flywheelsIO = flyWheelsIO;
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
        return new Shooter(new FlywheelsIO() {});
    }

    public static Shooter createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("using a dummy shooter on real robot", false);
        }
        return new Shooter(new FlywheelsIO() {}); // TODO
    }

    @Override
    public void periodic() {
        flywheelsIO.updateInputs(flywheelInputs);
        Logger.processInputs("Shooter/Flywheels", flywheelInputs);
        leftSpeedFilter.calculate(getLeftFlywheelSpeed());
        rightSpeedFilter.calculate(getRightFlywheelSpeed());

        if (RobotState.isDisabled()) {
            stopFlywheels();
        }
        if (Constants.isTuningMode()
                && (flywheelsKP.hasChanged(hashCode())
                        || flywheelsKI.hasChanged(hashCode())
                        || flywheelsKD.hasChanged(hashCode())
                        || flywheelsKV.hasChanged(hashCode())
                        || flywheelsKS.hasChanged(hashCode()))) {
            flywheelsIO.configPID(
                    flywheelsKP.get(), flywheelsKI.get(), flywheelsKD.get(), flywheelsKV.get(), flywheelsKS.get());
        }
    }

    public void setFlyWheelSpeeds(double leftSpeedRPM, double rightSpeedRPM) {
        leftFlywheelSetpointRPM = leftSpeedRPM;
        rightFlywheelSetpointRPM = rightSpeedRPM;
        leftSpeedFilter.reset();
        flywheelsIO.setSpeeds(leftSpeedRPM, rightSpeedRPM);
    }

    public void setFlywheelVolts(double rightVolts, double leftVolts) {
        flywheelsIO.setVoltage(MathUtil.clamp(rightVolts, -12, 12), MathUtil.clamp(leftVolts, -12, 12));
    }

    public Runnable stopFlywheels() {
        setFlywheelVolts(0, 0);
        return null;
    }

    public double getLeftFlywheelSpeed() {
        return flywheelInputs.leftVelocityRPM;
    }

    public double getRightFlywheelSpeed() {
        return flywheelInputs.rightVelocityRPM;
    }

    public boolean areFlywheelsSpunUp() {
        return MathUtil.isNear(
                        leftFlywheelSetpointRPM,
                        leftSpeedFilter.calculate(getLeftFlywheelSpeed()),
                        ShooterConstants.Flywheels.ERROR_TOLERANCE_RPM)
                && MathUtil.isNear(
                        rightFlywheelSetpointRPM,
                        rightSpeedFilter.calculate(getRightFlywheelSpeed()),
                        ShooterConstants.Flywheels.ERROR_TOLERANCE_RPM);
    }

    public double getSpinUpPercent() {
        return (getRightFlywheelSpeed() + getLeftFlywheelSpeed())
                / (getRightFlywheelSetpointRPM() + getLeftFlywheelSetpointRPM());
    }

    public Command spinUpCommand(Supplier<Double> leftSetPoint, Supplier<Double> rightSetPoint) {
        return Commands.run(() -> setFlyWheelSpeeds(leftSetPoint.get(), rightSetPoint.get()), this)
                .until(this::areFlywheelsSpunUp);
    }

    public Command stopShooterCommand() {
        return Commands.run(stopFlywheels());
    }

    @AutoLogOutput(key = "Shooter/Flywheels/leftSetPointRPM")
    public double getLeftFlywheelSetpointRPM() {
        return leftFlywheelSetpointRPM;
    }

    @AutoLogOutput(key = "Shooter/Flywheels/rightSetPointRPM")
    public double getRightFlywheelSetpointRPM() {
        return rightFlywheelSetpointRPM;
    }
}
