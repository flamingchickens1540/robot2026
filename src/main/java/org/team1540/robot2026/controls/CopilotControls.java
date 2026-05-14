package org.team1540.robot2026.controls;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class CopilotControls {
    public static final CopilotControls XboxController = new CopilotControls(CopilotControllerType.XBOX, 1);

    public final GenericHID hid;

    public final Trigger zeroTurret; // Zero turret to encoders
    public final Trigger zeroHood; // Zero hood to hard stop
    public final Trigger zeroIntake; // Zero intake to hard stop


    public final Trigger manualTurret; // Enable manual turret control
    public final DoubleSupplier manualTurretInput; // Manual turret control input;

    public final Trigger manualIntake; // Enable manual intake control
    public final DoubleSupplier manualIntakeInput; // Manual intake control input

    public final Trigger stowHood; // Stow hood

    public final Trigger lockTurret; // Enable turret locked mode

    public final Trigger zeroDriveOrientation; // Zero field orientation controls
    public final Trigger pointMode; // Point the intake in the translation direction

    public final Trigger stopTurret; // Stop turret movement and hold position

    public final Trigger trimShooterUp; // Increase shooter RPM setpoint
    public final Trigger trimShooterDown; // Decrease shooter RPM setpoint

    public final Trigger tuneShooter; // Run shooter tuning command
    public final Trigger tuningFeed; // Feed balls into shooter during tuning

    private CopilotControls(CopilotControllerType controllerType, int port) {
        switch (controllerType) {
            case XBOX -> {
                CommandXboxController controller = new CommandXboxController(port);
                hid = controller.getHID();

                zeroTurret = controller.back();
                zeroHood = controller.start();
                zeroIntake = controller.leftTrigger();

                manualTurret = controller.b();
                manualTurretInput = controller::getLeftX;

                manualIntake = controller.x();
                manualIntakeInput = controller::getRightY;

                stowHood = controller.a();

                pointMode = controller.leftBumper();

                lockTurret = controller.rightStick();

                zeroDriveOrientation = controller.rightTrigger();
                stopTurret = controller.rightBumper();

                trimShooterUp = controller.povUp();
                trimShooterDown = controller.povDown();

                tuneShooter = controller.y();
                tuningFeed = controller.leftStick();
            }
            default -> throw new IllegalArgumentException("Unexpected controller type: " + controllerType);
        }
    }

    private enum CopilotControllerType {
        XBOX
    }
}
