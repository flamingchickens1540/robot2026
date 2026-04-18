package org.team1540.robot2026.util.logging;

import edu.wpi.first.wpilibj.RobotController;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;

public class BatteryLogger {
    private static final BatteryInputsAutoLogged inputs = new BatteryInputsAutoLogged();

    private static double totalCurrent = 0.0;
    private static double totalPower = 0.0;
    private static double totalEnergy = 0.0;

    private static final Map<String, Double> subsystemCurrents = new HashMap<>();
    private static final Map<String, Double> subsystemPowers = new HashMap<>();
    private static final Map<String, Double> subsystemEnergies = new HashMap<>();

    public static void reportCurrent(String key, double... currents) {
        double current = Arrays.stream(currents).sum();
        double power = current * inputs.batteryVoltage;
        double energy = power * Constants.LOOP_PERIOD_SECS / 3600;

        totalCurrent += current;
        totalPower += power;
        totalEnergy += energy;

        subsystemCurrents.put(key, current);
        subsystemPowers.put(key, power);
        subsystemEnergies.merge(key, energy, Double::sum);

        String[] keys = key.split("/");
        if (keys.length < 2) {
            return;
        }

        String subkey = "";
        for (int i = 0; i < keys.length - 1; i++) {
            subkey += keys[i];
            if (i < keys.length - 2) {
                subkey += "/";
            }
            subsystemCurrents.merge(subkey, current, Double::sum);
            subsystemPowers.merge(subkey, power, Double::sum);
            subsystemEnergies.merge(subkey, energy, Double::sum);
        }
    }

    public static void periodic() {
        updateInputs();
        Logger.processInputs("Battery", inputs);

        reportCurrent("Controls/RoboRIO", inputs.rioCurrent);
        reportCurrent("Controls/CANcoders", 0.05 * 6);
        reportCurrent("Controls/Pigeon", 0.04);
        reportCurrent("Controls/CANivore", 0.03);
        reportCurrent("Controls/Radio", 0.5);

        Logger.recordOutput("BatteryLogger/TotalCurrent", totalCurrent, "amps");
        Logger.recordOutput("BatteryLogger/TotalPower", totalPower, "watts");
        Logger.recordOutput("BatteryLogger/TotalEnergy", totalEnergy, "watt hours");

        for (var entry : subsystemCurrents.entrySet()) {
            Logger.recordOutput("BatteryLogger/" + entry.getKey() + "/Current", entry.getValue(), "amps");
            subsystemCurrents.put(entry.getKey(), 0.0);
        }
        for (var entry : subsystemPowers.entrySet()) {
            Logger.recordOutput("BatteryLogger/" + entry.getKey() + "/Power", entry.getValue(), "watts");
            subsystemPowers.put(entry.getKey(), 0.0);
        }
        for (var entry : subsystemEnergies.entrySet()) {
            Logger.recordOutput("BatteryLogger/" + entry.getKey() + "/Energy", entry.getValue(), "watt hours");
        }

        resetTotals();
    }

    public static void resetTotals() {
        totalCurrent = 0.0;
        totalPower = 0.0;
    }

    @AutoLog
    public static class BatteryInputs {
        public double batteryVoltage = 0.0;
        public double rioCurrent = 0.0;
    }

    private static void updateInputs() {
        inputs.batteryVoltage = RobotController.getBatteryVoltage();
        inputs.rioCurrent = RobotController.getInputCurrent();
    }
}
