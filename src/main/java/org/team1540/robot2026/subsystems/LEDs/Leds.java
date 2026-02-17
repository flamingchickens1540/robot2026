package org.team1540.robot2026.subsystems.LEDs;

import static org.team1540.robot2026.subsystems.LEDs.LEDConstants.LED_LENGTH;
import static org.team1540.robot2026.subsystems.LEDs.LEDConstants.LED_PORT;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.team1540.robot2026.util.LoggedTracer;

public class Leds extends SubsystemBase {
    private final AddressableLED ledStrip = new AddressableLED(LED_PORT);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LED_LENGTH);

    private final List<LedWindow> children = new ArrayList<LedWindow>();

    public Leds() {
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.start();
    }

    public class LedWindow extends SubsystemBase {
        private final AddressableLEDBufferView view;
        private LEDPattern defaultpattern;
        private LEDPattern activepattern;

        private LedWindow(int start, int end) {
            view = ledBuffer.createView(start, end);
            children.add(this);
            this.setName("LedWindow " + start + " to " + end);
        }

        private void apply() {
            if (activepattern != null) {
                this.activepattern.applyTo(view);
            } else if (defaultpattern != null) {
                this.activepattern.applyTo(view);
            }
        }
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
