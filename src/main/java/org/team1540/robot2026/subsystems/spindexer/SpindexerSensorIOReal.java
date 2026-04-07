package org.team1540.robot2026.subsystems.spindexer;

import static org.team1540.robot2026.subsystems.spindexer.SpindexerConstants.LASER_CAN_ID;

import au.grapplerobotics.LaserCan;

public class SpindexerSensorIOReal implements SpindexerSensorIO {
    private final LaserCan laserCan = new LaserCan(LASER_CAN_ID);


    @Override
    public void updateInputs(SpindexerSensorIOInputs inputs) {
        inputs.isConnected = laserCan.getMeasurement() != null;
        if (!inputs.isConnected) inputs.distanceMM = laserCan.getMeasurement().distance_mm;

    }
}
