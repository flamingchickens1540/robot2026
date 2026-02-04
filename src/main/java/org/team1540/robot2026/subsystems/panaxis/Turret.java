package org.team1540.robot2026.subsystems.panaxis;

import static org.team1540.robot2026.subsystems.panaxis.TurretConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.util.LoggedTracer;
import org.team1540.robot2026.util.LoggedTunableNumber;

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

    private double setpointRotation;

    private final Debouncer zeroedDebouncer = new Debouncer(0.25);
    private boolean atZero = false;

    private Turret(TurretIO turretIO) {
        if (hasInstance) throw new IllegalStateException("Instance of elevator already exists");
        hasInstance = true;
        this.io = turretIO;
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
}
