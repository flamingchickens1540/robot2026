package org.team1540.robot2026.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2026.subsystems.intake.IntakeIO;

public class Intake extends SubsystemBase {

    private static boolean IntakeMotor = false;
    private static boolean hasInstance = false;
    private final IntakeIO intakeIO;
    private final IntakeIO movementIO;

    private final IntakeInputsAutoLogged positveInput = new IntakeInputsAutoLogged();
    private final IntakeInputsAutoLogged negativeInput = new IntakeInputsAutoLogged();
    private final Alert rollerDisconnectedAlert = new Alert("Intake roller disconnected", Alert.AlertType.kError);

    private Intake(IntakeIO intakeIO, IntakeIO movementIO) {
        if (hasInstance) throw new IllegalStateException("Intake already exists");
        {
        }
        hasInstance = true;
        this.intakeIO = intakeIO;
        this.movementIO = movementIO;
    }
}
