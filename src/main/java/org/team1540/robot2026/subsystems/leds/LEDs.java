package org.team1540.robot2026.subsystems.leds;

import static org.team1540.robot2026.subsystems.leds.LEDConstants.LED_PWM_PORT;
import static org.team1540.robot2026.subsystems.leds.LEDConstants.LED_STRIP_LENGTH;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final AddressableLED ledStrip = new AddressableLED(LED_PWM_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_STRIP_LENGTH);
    private final LEDPattern defaultPattern =
            LEDPattern.solid(new Color("#ff3700")).synchronizedBlink(RobotController::getRSLState);

    public LEDs() {
        ledStrip.setLength(buffer.getLength());
        ledStrip.start();
    }

    @Override
    public void periodic() {
        defaultPattern.applyTo(buffer);
        ledStrip.setData(buffer);
    }
}
