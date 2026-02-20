package org.team1540.robot2026;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2026.subsystems.Shooter.Shooter;

import java.util.function.Supplier;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);
    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    private final Shooter shooter;

    public RobotContainer() {
        configureButtonBindings();
        configureAutoRoutines();
        shooter = Shooter.createReal();
    }

    private void configureButtonBindings() {
        Supplier<Double> leftSetpoint = new Supplier<Double>() {
            @Override
            public Double get() {
                return 3000.0;
            }
        };
        Supplier<Double> rightSetpoint = new Supplier<Double>() {
            @Override
            public Double get() {
                return 3000.0;
            }
        };

        driver.y().whileTrue(shooter.spinUpCommand(leftSetpoint, rightSetpoint));
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
