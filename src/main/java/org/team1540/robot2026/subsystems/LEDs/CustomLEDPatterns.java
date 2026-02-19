package org.team1540.robot2026.subsystems.LEDs;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

import static org.team1540.robot2026.subsystems.LEDs.LEDConstants.RED_HEXSTRING;
import static org.team1540.robot2026.subsystems.LEDs.LEDConstants.YELLOW_HEXSTRING;

public class CustomLEDPatterns {
    public static LEDPattern red = LEDPattern.solid(new Color(RED_HEXSTRING)).synchronizedBlink(RobotController::getRSLState);
    //check red hexstring

    public static LEDPattern yellow = LEDPattern.solid(new Color(YELLOW_HEXSTRING)).synchronizedBlink(RobotController::getRSLState);
    //check yellow hexstring

    public static LEDPattern redYellow = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,new Color(RED_HEXSTRING),new Color(YELLOW_HEXSTRING));



}
