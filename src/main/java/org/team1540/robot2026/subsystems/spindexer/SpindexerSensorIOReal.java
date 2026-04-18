package org.team1540.robot2026.subsystems.spindexer;

import static org.team1540.robot2026.subsystems.spindexer.SpindexerConstants.LASER_CAN_ID;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import edu.wpi.first.wpilibj.DriverStation;

public class SpindexerSensorIOReal implements SpindexerSensorIO {
    private final LaserCan laserCan = new LaserCan(LASER_CAN_ID);

    public SpindexerSensorIOReal() {
        try {
            laserCan.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS);
            laserCan.setRangingMode(LaserCanInterface.RangingMode.SHORT);
        } catch (ConfigurationFailedException e) {
            DriverStation.reportError("Failed to configure LaserCan!", false);
        }
    }

    @Override
    public void updateInputs(SpindexerSensorIOInputs inputs) {
        inputs.isConnected = laserCan.getMeasurement() != null;
        if (inputs.isConnected) inputs.distanceMM = laserCan.getMeasurement().distance_mm;
    }
}
