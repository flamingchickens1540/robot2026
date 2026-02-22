package org.team1540.robot2026.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class JoystickUtil {
    public static double smartDeadzone(double rawInput, double deadzone) {
        double scalar = 1 / (1 - deadzone);
        if (rawInput > deadzone) {
            return (rawInput - deadzone) * scalar;
        }
        if (rawInput < -deadzone) {
            return (rawInput + deadzone) * scalar;
        }
        return 0;
    }

    public static double squaredSmartDeadzone(double rawInput, double deadzone) {
        double deadzonedInput = smartDeadzone(rawInput, deadzone);
        return Math.copySign(deadzonedInput * deadzonedInput, deadzonedInput);
    }

    public static Translation2d deadzonedJoystickTranslation(double rawX, double rawY, double deadzone) {
        double linearMagnitude = JoystickUtil.smartDeadzone(Math.hypot(rawX, rawY), deadzone);
        if (linearMagnitude < 1e-6) return Translation2d.kZero;
        Rotation2d linearDirection = new Rotation2d(rawX, rawY);
        return new Translation2d(linearMagnitude, linearDirection);
    }

    public static Translation2d squareDeadzonedJoystickTranslation(double rawX, double rawY, double deadzone) {
        double linearMagnitude = JoystickUtil.squaredSmartDeadzone(Math.hypot(rawX, rawY), deadzone);
        if (linearMagnitude < 1e-6) return Translation2d.kZero;
        Rotation2d linearDirection = new Rotation2d(rawX, rawY);
        return new Translation2d(linearMagnitude, linearDirection);
    }

    public static Command rumbleCommand(XboxController controller, double amount) {
        return Commands.startEnd(
                () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, amount),
                () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0));
    }
}
