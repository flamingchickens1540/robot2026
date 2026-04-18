package org.team1540.robot2026.subsystems.turret;

import static org.team1540.robot2026.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.FieldConstants;
import org.team1540.robot2026.MechanismVisualizer;
import org.team1540.robot2026.RobotState;
import org.team1540.robot2026.util.logging.BatteryLogger;
import org.team1540.robot2026.util.logging.LoggedTracer;
import org.team1540.robot2026.util.logging.LoggedTunableNumber;

public class Turret extends SubsystemBase {
    private static boolean hasInstance = false;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", KD);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", KV);

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private Rotation2d setpoint = Rotation2d.kZero;

    private boolean zeroed = false;

    @AutoLogOutput(key = "Turret/CRT/CalculatedPosition")
    private Rotation2d lastCalculatedCRTPosition = Rotation2d.kZero; // Last calculated position from CRT encoders

    @AutoLogOutput(key = "Turret/CRT/Error")
    private Rotation2d lastCRTError = Rotation2d.kZero; // Error between two CRT encoders

    @AutoLogOutput(key = "Turret/CRT/ZeroingError")
    private Rotation2d zeroingCRTError = Rotation2d.kZero; // Error between two CRT encoders at the moment of zeroing

    @AutoLogOutput(key = "Turret/CRT/MotorToCRTError")
    private Rotation2d lastMotorToCRTError = Rotation2d.kZero; // Error between motor encoder and CRT

    private final Alert motorDisconnectedAlert = new Alert("Turret motor disconnected", Alert.AlertType.kError);
    private final Alert smallEncoderDisconnectedAlert =
            new Alert("Turret " + SMALL_ENCODER_TEETH + "t encoder disconnected", Alert.AlertType.kError);
    private final Alert bigEncoderDisconnectedAlert =
            new Alert("Turret " + BIG_ENCODER_TEETH + "t encoder disconnected", Alert.AlertType.kError);

    private final Alert zeroingErrorAlert = new Alert(
            "Large encoder error of " + zeroingCRTError.getDegrees() + " degrees during zeroing",
            Alert.AlertType.kWarning);
    private final Alert unzeroedAlert = new Alert(
            "Large error between CRT position and motor position, press driver back button to rezero",
            Alert.AlertType.kError);

    public Turret(TurretIO turretIO) {
        if (hasInstance) throw new IllegalStateException("Instance of turret already exists");
        this.io = turretIO;
        hasInstance = true;

        setDefaultCommand(trackNearestHubCommand());

        CommandScheduler.getInstance().schedule(autoZeroOnStartupCommand());
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        if (DriverStation.isDisabled()) {
            stop();
            calculateTurretAngle();
            lastMotorToCRTError = Rotation2d.fromDegrees(
                    Math.abs(lastCalculatedCRTPosition.minus(inputs.position).getDegrees()));
        }

        RobotState.getInstance()
                .addTurretObservation(
                        getPosition(), Units.rotationsToRadians(getVelocityRPS()), inputs.positionTimestamp);
        MechanismVisualizer.addTurretData(inputs.position, setpoint);

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configFF(kS.get(), kV.get()), kS, kV);

        motorDisconnectedAlert.set(!inputs.connected);
        smallEncoderDisconnectedAlert.set(!inputs.smallEncoderConnected);
        bigEncoderDisconnectedAlert.set(!inputs.bigEncoderConnected);

        zeroingErrorAlert.setText("Large encoder error of " + zeroingCRTError.getDegrees() + " degrees during zeroing");
        zeroingErrorAlert.set(zeroingCRTError.getDegrees() > 5.0);

        unzeroedAlert.setText("Large error between CRT position and motor position of "
                + lastMotorToCRTError.getDegrees() + " deg, press driver back button to rezero");
        unzeroedAlert.set(!zeroed);

        Command activeCmd = CommandScheduler.getInstance().requiring(this);
        Logger.recordOutput(
                "Turret/ActiveCommand",
                activeCmd != null ? activeCmd.getName() + "_" + Integer.toHexString(activeCmd.hashCode()) : "None");

        BatteryLogger.reportCurrent("Turret", inputs.supplyCurrentAmps);

        LoggedTracer.record("Turret");
    }

    private Rotation2d calculateTurretAngle() {
        double smallEncoderPos = inputs.smallEncoderPosition.getRotations();
        double bigEncoderPos = inputs.bigEncoderPosition.getRotations();

        double[] smallEncoderPositions = new double[POSSIBLE_POS_ACC_DIGITS];
        double[] bigEncoderPositions = new double[POSSIBLE_POS_ACC_DIGITS];
        double out = 0;
        double minValue = 1;
        for (int i = 0; i < POSSIBLE_POS_ACC_DIGITS; i++) {
            smallEncoderPositions[i] = (i + (smallEncoderPos)) * SMALL_ENCODER_TEETH / MAIN_GEAR_TEETH; // 0 - 1
            bigEncoderPositions[i] = (i + (bigEncoderPos)) * BIG_ENCODER_TEETH / MAIN_GEAR_TEETH;
        }

        Logger.recordOutput("Turret/CRT/" + SMALL_ENCODER_TEETH + "tEncoderPositions", smallEncoderPositions);
        Logger.recordOutput("Turret/CRT/" + BIG_ENCODER_TEETH + "tEncoderPositions", bigEncoderPositions);

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

        Rotation2d position = Rotation2d.fromRotations(rawPosition.getRotations() - ANGLE_OFFSET.getRotations());
        lastCalculatedCRTPosition = position;
        return position;
    }

    private Rotation2d unwrapTurretAngle(Rotation2d targetAngle) {
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

    @AutoLogOutput(key = "Turret/IsZeroed")
    public boolean isZeroed() {
        return zeroed;
    }

    public void stop() {
        io.setVoltage(0);
    }

    @AutoLogOutput(key = "Turret/AtSetpoint")
    public boolean atSetpoint() {
        return atSetpoint(Rotation2d.fromDegrees(POS_ERR_TOLERANCE_DEGREES));
    }

    public boolean atSetpoint(Rotation2d tolerance) {
        return MathUtil.isNear(setpoint.getDegrees(), inputs.position.getDegrees(), tolerance.getDegrees());
    }

    @AutoLogOutput(key = "Turret/Setpoint")
    public Rotation2d getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(Rotation2d position) {
        setSetpoint(position, 0.0);
    }

    public void setSetpoint(Rotation2d position, double velocityRadPerSec) {
        setpoint = unwrapTurretAngle(position);
        io.setSetpoint(setpoint, KV * Units.radiansToRotations(velocityRadPerSec));
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

    public double getVelocityRPS() {
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
                                                        ? RobotState.getInstance()
                                                                .getRobotHeading()
                                                        : Rotation2d.kZero),
                                velocityRadPerSec.getAsDouble()),
                        this::stop)
                .withName("TurretSetpointCommand");
    }

    public Command trackNearestHubCommand() {
        Supplier<Translation2d> nearestHub = () -> {
            Pose2d pose = RobotState.getInstance().getTurretPose();
            return pose.getTranslation()
                    .nearest(List.of(
                            FieldConstants.Hub.topCenterPoint.toTranslation2d(),
                            FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()));
        };
        return commandToSetpoint(
                () -> nearestHub
                        .get()
                        .minus(RobotState.getInstance().getTurretPose().getTranslation())
                        .getAngle(),
                () -> {
                    Translation2d target = nearestHub.get();
                    Pose2d pose = RobotState.getInstance().getTurretPose();
                    ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldRelativeVelocity();
                    Translation2d turretVelocity = new Translation2d(
                                    robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
                            .plus(new Translation2d(
                                    robotVelocity.omegaRadiansPerSecond
                                            * ROBOT_TO_TURRET_2D
                                                    .getTranslation()
                                                    .getNorm(),
                                    pose.getRotation().rotateBy(Rotation2d.kCW_90deg)));
                    double targetDistance = pose.getTranslation().getDistance(target);

                    return -target.minus(pose.getTranslation())
                                            .rotateBy(Rotation2d.kCCW_90deg)
                                            .dot(turretVelocity)
                                    / (targetDistance * targetDistance)
                            - robotVelocity.omegaRadiansPerSecond;
                },
                true);
    }

    public Command zeroCommand() {
        return runOnce(this::stop)
                .andThen(() -> {
                    io.setMotorPosition(calculateTurretAngle());
                    zeroingCRTError = lastCRTError;
                    lastMotorToCRTError = Rotation2d.fromDegrees(Math.abs(
                            lastCalculatedCRTPosition.minus(inputs.position).getDegrees()));
                    zeroed = lastMotorToCRTError.getDegrees() < 1.0;
                })
                .ignoringDisable(true)
                .withName("TurretZeroCommand");
    }

    public Command autoZeroOnStartupCommand() {
        Timer stationaryTimer = new Timer();
        return Commands.runOnce(() -> {
                    if (getVelocityRPS() > 0.001) stationaryTimer.restart();
                })
                .andThen(zeroCommand()
                        .asProxy()
                        .andThen(stationaryTimer::restart)
                        .onlyIf(() -> stationaryTimer.hasElapsed(5.0)))
                .repeatedly()
                .beforeStarting(stationaryTimer::restart)
                .ignoringDisable(true)
                .until(DriverStation::isEnabled)
                .withName("TurretStartupAutoZeroCommand");
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
