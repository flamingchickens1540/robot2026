package org.team1540.robot2026.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Drivetrain extends SubsystemBase {
    private static boolean hasInstance = false;
    static final Lock odometryLock = new ReentrantLock();
}
