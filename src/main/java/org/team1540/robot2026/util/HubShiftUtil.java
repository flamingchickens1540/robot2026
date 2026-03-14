package org.team1540.robot2026.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class HubShiftUtil {
    public enum HubShift {
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME,
        AUTO,
        DISABLED
    }

    public record ShiftInfo(HubShift shift, double elapsedTime, double remainingTime, boolean active) {}

    private static final LoggedNetworkBoolean lostAutoOverride =
            new LoggedNetworkBoolean("SmartDashboard/HubShift/Lost Auto Override", false);
    private static final LoggedNetworkBoolean wonAutoOverride =
            new LoggedNetworkBoolean("SmartDashboard/HubShift/Won Auto Override", false);

    static {
        new Trigger(lostAutoOverride::get)
                .onTrue(Commands.runOnce(() -> wonAutoOverride.set(false)).ignoringDisable(true));
        new Trigger(wonAutoOverride::get)
                .onTrue(Commands.runOnce(() -> lostAutoOverride.set(false)).ignoringDisable(true));
    }

    public static final double autoEndTime = 20.0;
    public static final double teleopDuration = 140.0;
    private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
    private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};
    private static final boolean[] activeSchedule = {true, true, false, true, false, true};
    private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};
    private static final HubShift[] shiftsEnums = HubShift.values();
    private static final double timeResetThreshold = 3.0;

    private static final Timer shiftTimer = new Timer();
    private static double shiftTimerOffset = 0.0;

    private static final Alert invalidFMSDataAlert =
            new Alert("Auto winner not set by FMS! Use manual overrides.", Alert.AlertType.kError);

    public static void initialize() {
        shiftTimerOffset = 0;
        shiftTimer.restart();
    }

    public static void periodic() {
        ShiftInfo shiftInfo = getShiftInfo();
        boolean validFMSData = hasValidFMSData();

        invalidFMSDataAlert.set(!validFMSData && DriverStation.isFMSAttached() && DriverStation.isTeleopEnabled());

        Logger.recordOutput("HubShift/ShiftInfo", shiftInfo);
        Logger.recordOutput("HubShift/ValidFMSData", validFMSData);

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putString(
                "HubShift/Remaining Shift Time", String.format("%.1f", Math.max(shiftInfo.remainingTime(), 0)));
        SmartDashboard.putBoolean("HubShift/Hub Active", shiftInfo.active());
        SmartDashboard.putString("HubShift/Game State", shiftInfo.shift().name());
        SmartDashboard.putBoolean(
                "HubShift/Active First?",
                getFirstActiveAlliance() == DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    public static Alliance getFirstActiveAlliance() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        Optional<Boolean> winOverride = getAllianceWinOverride();
        if (winOverride.isPresent()) {
            if (winOverride.get()) return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
            else return alliance;
        }

        String message = DriverStation.getGameSpecificMessage();
        if (message != null && !message.isEmpty()) {
            if (message.charAt(0) == 'B') return Alliance.Red;
            else if (message.charAt(0) == 'R') return Alliance.Blue;
        }

        return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }

    public static ShiftInfo getShiftInfo() {
        boolean[] currentSchedule = getSchedule();
        double timerValue = shiftTimer.get();
        double currentTime = timerValue - shiftTimerOffset;
        double stateTimeElapsed = currentTime;
        double stateTimeRemaining = 0.0;
        boolean active = false;
        HubShift currentShift = HubShift.DISABLED;
        double fieldTeleopTime = teleopDuration - DriverStation.getMatchTime();

        if (DriverStation.isAutonomousEnabled()) {
            stateTimeElapsed = currentTime;
            stateTimeRemaining = autoEndTime - currentTime;
            active = true;
            currentShift = HubShift.AUTO;
        } else if (DriverStation.isEnabled()) {
            // Adjust the current offset if the time difference above the threshold
            if (Math.abs(fieldTeleopTime - currentTime) >= timeResetThreshold
                    && fieldTeleopTime <= 135
                    && DriverStation.isFMSAttached()) {
                shiftTimerOffset += currentTime - fieldTeleopTime;
                currentTime = timerValue + shiftTimerOffset;
            }
            int shiftIndex = -1;
            for (int i = 0; i < shiftStartTimes.length; i++) {
                if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
                    shiftIndex = i;
                    break;
                }
            }
            if (shiftIndex < 0) {
                // After last shift, so assume endgame
                shiftIndex = shiftStartTimes.length - 1;
            }

            // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
            stateTimeElapsed = currentTime - shiftStartTimes[shiftIndex];
            stateTimeRemaining = shiftEndTimes[shiftIndex] - currentTime;

            // If the state is the same as the last shift, combine the elapsed time
            if (shiftIndex > 0) {
                if (currentSchedule[shiftIndex] == currentSchedule[shiftIndex - 1]) {
                    stateTimeElapsed = currentTime - shiftStartTimes[shiftIndex - 1];
                }
            }

            // If the state is the same as the next shift, combine the remaining time
            if (shiftIndex < shiftEndTimes.length - 1) {
                if (currentSchedule[shiftIndex] == currentSchedule[shiftIndex + 1]) {
                    stateTimeRemaining = shiftEndTimes[shiftIndex + 1] - currentTime;
                }
            }

            active = currentSchedule[shiftIndex];
            currentShift = shiftsEnums[shiftIndex];
        }

        return new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
    }

    public static boolean hasValidFMSData() {
        String message = DriverStation.getGameSpecificMessage();
        return message != null && !message.isEmpty() && (message.charAt(0) == 'B' || message.charAt(0) == 'R');
    }

    private static Optional<Boolean> getAllianceWinOverride() {
        if (lostAutoOverride.get()) return Optional.of(false);
        else if (wonAutoOverride.get()) return Optional.of(true);
        return Optional.empty();
    }

    private static boolean[] getSchedule() {
        return getFirstActiveAlliance() == DriverStation.getAlliance().orElse(Alliance.Blue)
                ? activeSchedule
                : inactiveSchedule;
    }
}
