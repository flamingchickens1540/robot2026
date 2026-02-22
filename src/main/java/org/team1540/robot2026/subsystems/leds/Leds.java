package org.team1540.robot2026.subsystems.leds;

import static org.team1540.robot2026.subsystems.leds.LEDConstants.LED_LENGTH;
import static org.team1540.robot2026.subsystems.leds.LEDConstants.LED_PORT;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.team1540.robot2026.util.LoggedTracer;

public class Leds extends SubsystemBase {
    private static final AddressableLED ledStrip = new AddressableLED(LED_PORT);
    private static final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
    private LEDPattern defaultPattern =
            LEDPattern.solid(new Color("#ff3700")).synchronizedBlink(RobotController::getRSLState);

    private static final List<LedWindow> children = new ArrayList<LedWindow>();
    public static final LedWindow viewFull = new LedWindow(0, LED_LENGTH - 1);
    public static final LedWindow viewTop = new LedWindow(LED_LENGTH - 6, LED_LENGTH - 1);

    public Leds() {
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.start();
        viewFull.setDefaultPattern(CustomLEDPatterns.movingRainbow);
    }

    public static Color getAllianceColor() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? Color.kBlue
                : Color.kRed;
    }

    public static class LedWindow extends SubsystemBase {
        private final AddressableLEDBufferView view;
        private LEDPattern defaultPattern;
        private LEDPattern activePattern;

        private LedWindow(int start, int end) {
            view = ledBuffer.createView(start, end);
            children.add(this);
            this.setName("LedWindow " + start + " to " + end);
        }

        private void apply() {
            if (activePattern != null) {
                this.activePattern.applyTo(view);
            } else if (defaultPattern != null) {
                this.activePattern.applyTo(view);
            }
        }

        public Command commandShowPattern(LEDPattern pattern) {
            return Commands.startEnd(() -> this.activePattern = pattern, () -> this.activePattern = null, this)
                    .ignoringDisable(true);
        }

        public Command commandShowPattern(Supplier<LEDPattern> pattern) {
            return Commands.startEnd(() -> this.activePattern = pattern.get(), () -> this.activePattern = null, this)
                    .ignoringDisable(true);
        }

        public void setDefaultPattern(LEDPattern pattern) {
            this.defaultPattern = pattern;
        }

        public Command commandDefaultPattern(Supplier<LEDPattern> pattern) {
            return Commands.runOnce(() -> this.setDefaultPattern(pattern.get())).ignoringDisable(true);
        }

        public Command showRSLState() {
            LEDPattern pattern = LEDPattern.solid(new Color("#ff3700"));
            return commandShowPattern(pattern.synchronizedBlink(RobotController::getRSLState));
        }

        @Override
        public void periodic() {
            LoggedTracer.reset();

            for (LedWindow child : children) {
                child.apply();
            }

            ledStrip.setData(ledBuffer);

            LoggedTracer.record("LEDs");
        }
    }
}
