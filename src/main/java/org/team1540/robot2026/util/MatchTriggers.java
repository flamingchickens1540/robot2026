package org.team1540.robot2026.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// TODO add triggers for time periods
public class MatchTriggers {
    public static Trigger endgame() {
        return timeRemaining(30);
    }

    public static Trigger timeRemaining(double secs) {
        return new Trigger(() -> DriverStation.getMatchTime() <= secs)
                .and(DriverStation::isTeleopEnabled)
                .and(DriverStation::isFMSAttached);
    }
}
