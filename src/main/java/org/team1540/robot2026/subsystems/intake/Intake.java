package org.team1540.robot2026.subsystems.intake;

import static org.team1540.robot2026.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.util.LoggedTracer;
import org.team1540.robot2026.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {

    private static boolean hasInstance = false;

    public enum IntakeState {
        STOW(new LoggedTunableNumber("Intake/Setpoints/Stow/AngleDegrees", PIVOT_MIN_ANGLE.getDegrees())),
        INTAKE(new LoggedTunableNumber("Intake/Setpoints/Intake/AngleDegrees", PIVOT_MAX_ANGLE.getDegrees()));

        private final DoubleSupplier pivotPosition;

        IntakeState(DoubleSupplier pivotPositionDeg) {
            this.pivotPosition = pivotPositionDeg;
        }

        public Rotation2d pivotPosition() {
            return Rotation2d.fromDegrees(pivotPosition.getAsDouble());
        }
    }

    private final IntakeIO io;
    private final LoggedTunableNumber pivotKP = new LoggedTunableNumber("Intake/kP", PIVOT_KP);
    private final LoggedTunableNumber pivotKI = new LoggedTunableNumber("Intake/kI", PIVOT_KI);
    private final LoggedTunableNumber pivotKD = new LoggedTunableNumber("Intake/kD", PIVOT_KD);
    private final LoggedTunableNumber pivotKS = new LoggedTunableNumber("Intake/kS", PIVOT_KS);
    private final LoggedTunableNumber pivotKV = new LoggedTunableNumber("Intake/kV", PIVOT_KV);
    private final LoggedTunableNumber pivotKG = new LoggedTunableNumber("Intake/kG", PIVOT_KG);

    private final Alert pivotDisconnectedAlert = new Alert("Intake pivot disconnected", Alert.AlertType.kError);
    private final Alert rollerDisconnectedAlert = new Alert("Intake roller disconnected", Alert.AlertType.kError);

    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    private Rotation2d pivotSetpoint = PIVOT_MIN_ANGLE;

    private Intake(IntakeIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of intake already exists");
        hasInstance = true;
        this.io = io;
    }

    public void periodic() {
        LoggedTracer.reset();

        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // MechanismVisualizer.getInstance().setIntakeRotation(inputs.pivotPosition);

        if (DriverStation.isDisabled()) stopAll();

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setPivotPID(pivotKP.get(), pivotKI.get(), pivotKD.get()),
                pivotKP,
                pivotKI,
                pivotKD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setPivotFF(pivotKS.get(), pivotKV.get(), pivotKG.get()),
                pivotKS,
                pivotKV,
                pivotKG);

        pivotDisconnectedAlert.set(!inputs.pivotConnected);
        rollerDisconnectedAlert.set(!inputs.spinConnected);

        LoggedTracer.record("Intake");
    }

    public void setRollerVoltage(double voltage) {
        io.setIntakeVoltage(voltage);
    }

    public void setPivotSetpoint(Rotation2d rotations) {
        pivotSetpoint = rotations;
        io.setPivotSetpoint(rotations);
    }

    public void resetPivotPosition(Rotation2d position) {
        io.resetPivotPosition(position);
    }

    public Rotation2d getPivotPosition() {
        return inputs.pivotPosition;
    }

    public double getPivotVelocityRPS() {
        return inputs.pivotMotorVelocityRPS;
    }

    public void setPivotVoltage(double voltage) {
        io.setPivotVoltage(voltage);
    }

    public void stopAll() {
        setRollerVoltage(0);
        setPivotVoltage(0);
    }

    public void holdPivot() {
        setPivotSetpoint(inputs.pivotPosition);
    }

    @AutoLogOutput(key = "Intake/PivotAtSetpoint")
    public boolean isPivotAtSetpoint() {
        return MathUtil.isNear(pivotSetpoint.getDegrees(), inputs.pivotPosition.getDegrees(), 3.0);
    }

    @AutoLogOutput(key = "Intake/PivotSetpoint")
    public Rotation2d getPivotSetpoint() {
        return pivotSetpoint;
    }

    public Command commandToSetpoint(IntakeState state) {
        return (Commands.run(() -> setPivotSetpoint(state.pivotPosition()), this)
                        .until(this::isPivotAtSetpoint))
                .handleInterrupt(this::holdPivot);
    }

    public Command commandRunIntake(double percent) {
        return Commands.startEnd(() -> this.setRollerVoltage(percent * 12), () -> this.setRollerVoltage(0), this);
    }

    public Command zeroCommand() {
        return runOnce(() -> setPivotVoltage(-2))
                .andThen(
                        Commands.waitUntil(new Trigger(() -> inputs.pivotStatorCurrentAmps >= 20).debounce(0.5)),
                        runOnce(() -> resetPivotPosition(PIVOT_MIN_ANGLE)));
    }

    public static Intake createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real coral intake on simulated robot", false);
        }
        return new Intake(new IntakeIOTalonFX());
    }

    public static Intake createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated coral intake on real robot", false);
        }
        return new Intake(new IntakeIOSim());
    }

    public static Intake createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy coral intake on real robot", false);
        }
        return new Intake(new IntakeIO() {});
    }
}
