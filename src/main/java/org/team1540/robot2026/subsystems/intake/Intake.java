package org.team1540.robot2026.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2026.subsystems.intake.IntakeIO;
import org.team1540.robot2026.subsystems.intake.IntakeInputsAutoLogged;

import java.util.logging.Logger;

public class Intake extends SubsystemBase {

    private static boolean IntakeMotor = false;
    private static boolean hasInstance = false;
    private final IntakeIO intakeIO;
    private final IntakeIO movementIO;

    private final IntakeInputsAutoLogged positiveInput = new IntakeInputsAutoLogged();
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

    public Command intake() {
        return Commands.run(() -> {
            intakeIO.setIntakeVoltage(12);
            movementIO.setIntakeVoltage(-12);
        }).finallyDo(() -> {
            intakeIO.setIntakeVoltage(0);
            movementIO.setIntakeVoltage(0);
        });
    }

    public Command reverseIntake() {
        return Commands.run(() -> {
            intakeIO.setIntakeVoltage(-12);
            movementIO.setIntakeVoltage(12);
        }).finallyDo(() -> {
            intakeIO.setIntakeVoltage(0);
            movementIO.setIntakeVoltage(0);
        });
    }

    public Command stop() {
        return Commands.runOnce(() -> {
            intakeIO.setIntakeVoltage(0);
            movementIO.setIntakeVoltage(0);
        });
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(positiveInput);
        movementIO.updateInputs(negativeInput);
        //Logger.process
    }
}

