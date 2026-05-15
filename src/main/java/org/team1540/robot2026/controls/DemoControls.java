package org.team1540.robot2026.controls;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1540.robot2026.util.hid.CommandEnvisionController;

import java.util.function.DoubleSupplier;

public class DemoControls {

    public static final DemoControls EnvisionController = new DemoControls(DemoControls.DemoControllerType.ENVISION, 3);
    public static final DemoControls XboxController = new DemoControls(DemoControls.DemoControllerType.XBOX, 3);

    public final GenericHID hid;

    public final DoubleSupplier driveX; // Drive forward/back
    public final DoubleSupplier driveY; // Drive left/right
    public final DoubleSupplier driveRotation; // Drive rotation


    public final Trigger intake; // Run intake
    public final Trigger shoot; // Shoot
    public final DoubleSupplier manualTurretInput; // Manual turret control input;




    private DemoControls(DemoControllerType controllerType, int port) {
        switch (controllerType) {
            case ENVISION -> {
                CommandEnvisionController controller = new CommandEnvisionController(port);
                hid = controller.getHID();
                Trigger enabled = CopilotControls.XboxController.disableBuddy;

                driveX = () -> enabled.getAsBoolean() ? 0.0 : -controller.getLeftY()*0.5;
                driveY = () -> enabled.getAsBoolean() ? 0.0 : -controller.getLeftX()*0.5;
                driveRotation = () -> enabled.getAsBoolean() ? 0.0 : -controller.getRightX()*0.5;


                intake = enabled.and(controller.x());
                shoot = enabled.and(controller.b());


                manualTurretInput = ()-> enabled.getAsBoolean() ? 0.0 : controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();

            }
            case XBOX -> {
                CommandXboxController controller = new CommandXboxController(port);
                hid = controller.getHID();
                Trigger enabled = CopilotControls.XboxController.disableBuddy;

                driveX = () -> enabled.getAsBoolean() ? 0.0 : -controller.getLeftY()*0.5;
                driveY = () -> enabled.getAsBoolean() ? 0.0 : -controller.getLeftX()*0.5;
                driveRotation = () -> enabled.getAsBoolean() ? 0.0 : -controller.getRightX()*0.5;


                intake = enabled.and(controller.x());
                shoot = enabled.and(controller.b());


                manualTurretInput = ()-> enabled.getAsBoolean() ? 0.0 : controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
            }
            default -> throw new IllegalStateException("Unexpected controller type: " + controllerType);
        }
    }

    private enum DemoControllerType {
        ENVISION,
        XBOX
    }
}

