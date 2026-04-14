package org.team1540.robot2026.subsystems.leds;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;
import static org.team1540.robot2026.subsystems.leds.LEDConstants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Map;
import java.util.function.DoubleSupplier;
import org.team1540.robot2026.RobotState;
import org.team1540.robot2026.subsystems.drive.DrivetrainConstants;
import org.team1540.robot2026.subsystems.hood.HoodConstants;

public class CustomLEDPatterns {
    private static final Time DEFAULT_STROBE_DURATION = Seconds.of(0.07);

    public static LEDPattern movingRainbow(Frequency velocity) {
        return movingRainbow(velocity, 255, 255);
    }

    public static LEDPattern movingRainbow(Frequency velocity, int saturation, int value) {
        final double periodMicros = velocity.asPeriod().in(Microseconds);

        return (reader, writer) -> {
            final double step = 180.0 / reader.getLength();
            long now = RobotController.getTime();

            double t = (now / periodMicros);
            for (int i = 0; i < reader.getLength(); i++) {
                int offset = (int) (i * step + t * 180) % 180;
                writer.setHSV(i, offset, saturation, value);
            }
        };
    }

    public static LEDPattern strobe(Color color, Time duration) {
        return LEDPattern.solid(color).blink(duration);
    }

    public static LEDPattern strobe(Color color) {
        return strobe(color, DEFAULT_STROBE_DURATION);
    }

    public static LEDPattern drivetrainSpeed(Color color) {
        return LEDPattern.solid(color).mask(LEDPattern.progressMaskLayer(() -> {
            ChassisSpeeds robotSpeed = RobotState.getInstance().getRobotVelocity();
            return Math.hypot(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
                    / DrivetrainConstants.MAX_LINEAR_SPEED_MPS;
        }));
    }

    public static LEDPattern hoodAngle(Color color, DoubleSupplier hoodAngleDegrees) {
        return LEDPattern.solid(color)
                .mask(LEDPattern.progressMaskLayer(
                        () -> hoodAngleDegrees.getAsDouble() / HoodConstants.MAX_ANGLE.getDegrees()));
    }

    public static LEDPattern transFlag() {
        return LEDPattern.steps(
                Map.of(
                        0, new Color(91, 206, 250),
                        0.2, new Color(245, 169, 184),
                        0.4, Color.kWhite,
                        0.6, new Color(245, 169, 184),
                        0.8, new Color(91, 206, 250)
                ));
    }
}
