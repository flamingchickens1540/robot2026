package org.team1540.robot2026.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    private double feederSetpointRPM = 0.0;


    private static boolean hasInstance = false;

    private Spindexer(SpindexerIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of indexer already exists");
        hasInstance = true;
        this.io = io;
    }
}
