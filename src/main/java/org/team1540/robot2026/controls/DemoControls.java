package org.team1540.robot2026.controls;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1540.robot2026.util.hid.CommandEnvisionController;

import java.util.function.DoubleSupplier;

public class DemoControls {

    public static final DemoControls EnvisionController = new DemoControls(DemoControls.DemoControllerType.ENVISION, 0);
    public static final DemoControls XboxController = new DemoControls(DemoControls.DemoControllerType.XBOX, 0);

    public final GenericHID hid;

    public final DoubleSupplier driveX; // Drive forward/back
    public final DoubleSupplier driveY; // Drive left/right
    public final DoubleSupplier driveRotation; // Drive rotation

    public final Trigger zeroDriveOrientation; // Zero field orientation controls
    public final Trigger pointMode; // Point the intake in the translation direction
    public final Trigger driveXMode; // Move wheels to X configuration to resist being pushed

    public final Trigger intake; // Run intake
    public final Trigger shoot; // Shoot
    public final Trigger forceShoot; // Shoot without shooter position or robot position checks

    public final Trigger outtake; // Reverse intake
    public final Trigger stowIntake; // Stow intake

    public final Trigger stowHood; // Stow hood
    public final Trigger stopTurret; // Stop turret movement and hold position
    public final Trigger zeroTurret; // Zero turret to encoders

    private DemoControls(DemoControllerType controllerType, int port) {
        switch (controllerType) {
            case ENVISION -> {
                CommandEnvisionController controller = new CommandEnvisionController(port);
                hid = controller.getHID();

                driveX = () -> -controller.getLeftY()*0.5;
                driveY = () -> -controller.getLeftX()*0.5;
                driveRotation = () -> -controller.getRightX()*0.5;

                zeroDriveOrientation = controller.start();
                pointMode = controller.a();
                driveXMode = controller.x();

                intake = controller.leftTrigger();
                shoot = controller.rightTrigger();
                forceShoot = controller.rightBumper();

                outtake = controller.leftInnerPaddle();
                stowIntake = controller.leftOuterPaddle();

                stowHood = controller.rightOuterPaddle();
                stopTurret = controller.leftBumper();
                zeroTurret = controller.back();
            }
            case XBOX -> {
                CommandXboxController controller = new CommandXboxController(port);
                hid = controller.getHID();

                driveX = () -> -controller.getLeftY()*0.5;
                driveY = () -> -controller.getLeftX()*0.5;
                driveRotation = () -> -controller.getRightX()*0.5;

                zeroDriveOrientation = controller.start();
                pointMode = controller.a();
                driveXMode = controller.x();

                intake = controller.leftTrigger();
                shoot = controller.rightTrigger();
                forceShoot = controller.rightBumper();

                outtake = controller.povDown();
                stowIntake = controller.leftStick();

                stowHood = controller.rightStick();
                stopTurret = controller.leftBumper();
                zeroTurret = controller.back();
            }
            default -> throw new IllegalStateException("Unexpected controller type: " + controllerType);
        }
    }

    private enum DemoControllerType {
        ENVISION,
        XBOX
    }
}

