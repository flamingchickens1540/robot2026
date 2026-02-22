package org.team1540.robot2026.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static org.team1540.robot2026.subsystems.leds.LEDConstants.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

public class CustomLEDPatterns {
    public static LEDPattern red =
            LEDPattern.solid(new Color(RED_HEXSTRING)).synchronizedBlink(RobotController::getRSLState);
    // check red hexstring

    public static LEDPattern yellow =
            LEDPattern.solid(new Color(YELLOW_HEXSTRING)).synchronizedBlink(RobotController::getRSLState);
    // check yellow hexstring

    public static LEDPattern flash = LEDPattern.solid(Color.kAntiqueWhite);

    public static LEDPattern purple = LEDPattern.solid(Color.kPurple);

    public static LEDPattern green = LEDPattern.solid(Color.kGreen);

    public static LEDPattern orange = LEDPattern.solid(Color.kOrange);

    public static LEDPattern blue = LEDPattern.solid(Color.kBlue);

    public static LEDPattern movingRainbow = LEDPattern.rainbow(255, 255)
            .scrollAtRelativeSpeed(Percent.per(Second).of(25));
}
