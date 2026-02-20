package org.team1540.robot2026;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    public CommandXboxController driver = new CommandXboxController(0);

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        configureButtonBindings();
        configureAutoRoutines();
    }

    private void configureButtonBindings() {
        /* driver.leftTrigger().whileTrue(Intake.IntakeState.INTAKE).
        commandToSetpoint(0) */
    }

    private void configureAutoRoutines() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
