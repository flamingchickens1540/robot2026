package org.team1540.robot2026.subsystems.panaxis;

import static org.team1540.robot2026.subsystems.panaxis.TurretConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.util.LoggedTracer;
import org.team1540.robot2026.util.LoggedTunableNumber;

import java.util.function.DoubleSupplier;

public class Turret extends SubsystemBase {
    private boolean hasInstance = false;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", KD);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", KV);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Turret/kG", KG);

    private final Alert motorDisconnectedAlert = new Alert("Turret motor disconnected", Alert.AlertType.kError);

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private CANcoder mainCancoder;
    private CANcoder secondaryCancoder;


    private double setpointRotation;

    private final Debouncer zeroedDebouncer = new Debouncer(0.25);
    private boolean atZero = false;

    private Turret(TurretIO turretIO) {
        if (hasInstance) throw new IllegalStateException("Instance of elevator already exists");
        hasInstance = true;
        this.io = turretIO;
        mainCancoder = new CANcoder(MAIN_CANCODER_ID);
        secondaryCancoder = new CANcoder(SECONDARY_CANCODER_ID);
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        if (RobotState.isDisabled()) stop();

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configFF(kS.get(), kV.get(), kG.get()), kS, kV, kG);

        // MechanismVisualizer.getInstance().setElevatorPosition(inputs.positionMeters[0]); TODO set up sim pos

        motorDisconnectedAlert.set(!inputs.driveConnected);

        LoggedTracer.record("Turret");
    }

    private void stop() {
        io.setVoltage(0);
    }

    @AutoLogOutput(key = "Turret/AtSetpoint")
    public boolean isAtSetpoint() {
        return MathUtil.isNear(Units.degreesToRadians(setpointRotation), inputs.drivePositionRads, POS_ERR_TOLERANCE_DEGREES);
    }

    @AutoLogOutput(key = "Turret/Setpoint")
    public double getSetpoint() {
        return setpointRotation;
    }

    public void setRotation(double rotationDegrees) {
        rotationDegrees = MathUtil.clamp(rotationDegrees, -MAX_TURRET_ROTATION, MAX_TURRET_ROTATION);
        setpointRotation = rotationDegrees;
        io.setRotation(setpointRotation);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    /* TODO
    public double timeToSetpoint(double setpoint) {
        trapezoidProfile.calculate(
                0.0,
                new TrapezoidProfile.State(getPosition(), inputs.velocityMPS[0]),
                new TrapezoidProfile.State(setpoint, 0));
        return trapezoidProfile.totalTime();
    }
    */

    public double getPosition() {
        return inputs.drivePositionRads;
    }

    public double getVelocity() {
        return inputs.driveVelocityRadPerSec;
    }

    public void setBrakeMode(boolean isBrakeMode) {
        io.setBrakeMode(isBrakeMode);
    }

    public void holdPosition() {
        setRotation(inputs.drivePositionRads);
    }

    public double calculateTurretRotation() {
        double mainPos = mainCancoder.getAbsolutePosition().getValueAsDouble();
        double secondaryPos = secondaryCancoder.getAbsolutePosition().getValueAsDouble();
        return 0;
    }

    public Command commandToSetpoint(DoubleSupplier rotationDegrees) {
        return Commands.runOnce(() -> setRotation(rotationDegrees.getAsDouble()),this
        );
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
