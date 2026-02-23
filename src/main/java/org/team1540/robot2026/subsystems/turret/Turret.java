package org.team1540.robot2026.subsystems.turret;

import static org.team1540.robot2026.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.RobotState;
import org.team1540.robot2026.util.LoggedTracer;
import org.team1540.robot2026.util.LoggedTunableNumber;

public class Turret extends SubsystemBase {
    private static boolean hasInstance = false;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", KD);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", KV);

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private Rotation2d setpointRotation = Rotation2d.kZero;

    @AutoLogOutput(key = "Turret/CRT/Error")
    private Rotation2d lastCRTError = Rotation2d.kZero;

    @AutoLogOutput(key = "Turret/CRT/ZeroingError")
    private Rotation2d zeroingCRTError = Rotation2d.kZero;

    private final Alert motorDisconnectedAlert = new Alert("Turret motor disconnected", Alert.AlertType.kError);
    private final Alert smallEncoderDisconnectedAlert =
            new Alert("Turret " + SMALL_ENCODER_GEAR_TOOTH_COUNT + "t encoder disconnected", Alert.AlertType.kError);
    private final Alert bigEncoderDisconnectedAlert =
            new Alert("Turret " + BIG_ENCODER_GEAR_TOOTH_COUNT + "t encoder disconnected", Alert.AlertType.kError);
    private final Alert zeroingErrorAlert = new Alert(
            "Large encoder error of " + zeroingCRTError.getDegrees() + " degrees during zeroing",
            Alert.AlertType.kWarning);

    public Turret(TurretIO turretIO) {
        if (hasInstance) throw new IllegalStateException("Instance of turret already exists");
        this.io = turretIO;
        hasInstance = true;
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        if (DriverStation.isDisabled()) {
            stop();
            Logger.recordOutput("Turret/CRT/CalculatedPosition", calculateTurretAngle());
        }

        RobotState.getInstance().addTurretObservation(getPosition(), inputs.positionTimestamp);

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configFF(kS.get(), kV.get()), kS, kV);

        motorDisconnectedAlert.set(!inputs.connected);
        smallEncoderDisconnectedAlert.set(!inputs.smallEncoderConnected);
        bigEncoderDisconnectedAlert.set(!inputs.bigEncoderConnected);
        zeroingErrorAlert.setText("Large encoder error of " + zeroingCRTError.getDegrees() + " degrees during zeroing");
        zeroingErrorAlert.set(zeroingCRTError.getDegrees() > 5.0);

        LoggedTracer.record("Turret");
    }

    public Rotation2d calculateTurretAngle() {
        double smallEncoderPos = inputs.smallEncoderPosition.getRotations();
        double bigEncoderPos = inputs.bigEncoderPosition.getRotations();

        double[] smallEncoderPositions = new double[POSSIBLE_POS_ACC_DIGITS];
        double[] bigEncoderPositions = new double[POSSIBLE_POS_ACC_DIGITS];
        double out = 0;
        double minValue = 1;
        for (int i = 0; i < POSSIBLE_POS_ACC_DIGITS; i++) {
            smallEncoderPositions[i] =
                    (i + (smallEncoderPos)) * SMALL_ENCODER_GEAR_TOOTH_COUNT / DRIVEN_GEAR_TOOTH_COUNT; // 0 - 1
            bigEncoderPositions[i] = (i + (bigEncoderPos)) * BIG_ENCODER_GEAR_TOOTH_COUNT / DRIVEN_GEAR_TOOTH_COUNT;
        }

        Logger.recordOutput(
                "Turret/CRT/" + SMALL_ENCODER_GEAR_TOOTH_COUNT + "tEncoderPositions", smallEncoderPositions);
        Logger.recordOutput("Turret/CRT/" + BIG_ENCODER_GEAR_TOOTH_COUNT + "tEncoderPositions", bigEncoderPositions);

        for (int i = 0; i < POSSIBLE_POS_ACC_DIGITS; i++) {
            for (int z = 0; z < POSSIBLE_POS_ACC_DIGITS; z++) {
                if (Math.abs(smallEncoderPositions[i] - bigEncoderPositions[z]) < minValue) {
                    out = (smallEncoderPositions[i] + bigEncoderPositions[z]) / 2;
                    minValue = Math.abs(smallEncoderPositions[i] - bigEncoderPositions[z]);
                }
            }
        }
        lastCRTError = Rotation2d.fromRotations(minValue);

        Rotation2d rawPosition = Rotation2d.fromRotations(out);
        Logger.recordOutput("Turret/CRT/RawPosition", rawPosition);

        return Rotation2d.fromRotations(rawPosition.getRotations() - ANGLE_OFFSET.getRotations());
    }

    public Rotation2d unwrapTurretAngle(Rotation2d targetAngle) {
        double targetRot = targetAngle.plus(Rotation2d.kZero).getRotations();
        double currentRot = getPosition().getRotations();
        double bestRot = 0.0;
        boolean hasBestRot = false;
        for (int i = -2; i <= 2; i++) {
            double candidate = targetRot + i;
            if (candidate < MIN_ANGLE.getRotations() || candidate > MAX_ANGLE.getRotations()) {
                continue;
            }
            if (!hasBestRot || Math.abs(candidate - currentRot) < Math.abs(bestRot - currentRot)) {
                hasBestRot = true;
                bestRot = candidate;
            }
        }
        return Rotation2d.fromRotations(bestRot);
    }

    public void stop() {
        io.setVoltage(0);
    }

    @AutoLogOutput(key = "Turret/AtSetpoint")
    public boolean atSetpoint() {
        return MathUtil.isNear(setpointRotation.getDegrees(), inputs.position.getDegrees(), POS_ERR_TOLERANCE_DEGREES);
    }

    @AutoLogOutput(key = "Turret/Setpoint")
    public Rotation2d getSetpoint() {
        return setpointRotation;
    }

    public void setSetpoint(Rotation2d position) {
        setSetpoint(position, 0.0);
    }

    public void setSetpoint(Rotation2d position, double velocityRadPerSec) {
        setpointRotation = unwrapTurretAngle(position);
        io.setSetpoint(setpointRotation, velocityRadPerSec);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public Rotation2d getPosition() {
        return inputs.position;
    }

    public Rotation2d getFieldRelativePosition() {
        return getPosition().plus(RobotState.getInstance().getRobotHeading());
    }

    public double getVelocity() {
        return inputs.velocityRPS;
    }

    public void setBrakeMode(boolean isBrakeMode) {
        io.setBrakeMode(isBrakeMode);
    }

    public Command commandToSetpoint(
            Supplier<Rotation2d> rotation, DoubleSupplier velocityRadPerSec, boolean isFieldRelative) {
        return runEnd(
                () -> setSetpoint(
                        rotation.get()
                                .minus(
                                        isFieldRelative
                                                ? RobotState.getInstance().getRobotHeading()
                                                : Rotation2d.kZero),
                        velocityRadPerSec.getAsDouble()),
                this::stop);
    }

    public Command zeroCommand() {
        return runOnce(this::stop).andThen(runOnce(() -> {
            io.setMotorPosition(calculateTurretAngle());
            zeroingCRTError = lastCRTError;
        }));
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
