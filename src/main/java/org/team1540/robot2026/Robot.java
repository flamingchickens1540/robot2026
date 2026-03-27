package org.team1540.robot2026;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private final RobotContainer robotContainer;

    public Robot() {
        super(Constants.LOOP_PERIOD_SECS);

        if (Constants.isTuningMode()) {
            CanBridge.runTCP();
        }
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });
        Logger.recordMetadata("TuningMode", Constants.isTuningMode() ? "on" : "off");

        // Set up data receivers & replay source
        switch (Constants.CURRENT_MODE) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.addDataReceiver(new LogDataReceiver() {
            @Override
            public void start() {
                Threads.setCurrentThreadPriority(true, 1);
            }

            @Override
            public void putTable(LogTable table) {}
        });

        // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
        // Logger.disableDeterministicTimestamps()

        // Start AdvantageKit logger
        Logger.start();

        // Silence Rotation2d warnings
        MathShared mathShared = MathSharedStore.getMathShared();
        MathSharedStore.setMathShared(new MathShared() {
            @Override
            public void reportError(String error, StackTraceElement[] stackTrace) {
                if (error.startsWith("x and y components of Rotation2d are zero")) {
                    return;
                }
                mathShared.reportError(error, stackTrace);
            }

            @Override
            public void reportUsage(MathUsageId id, int count) {
                mathShared.reportUsage(id, count);
            }

            @Override
            public double getTimestamp() {
                return mathShared.getTimestamp();
            }
        });

        // Set up command logging
        CommandScheduler.getInstance()
                .onCommandInitialize(cmd -> Logger.recordOutput(
                        "Commands/" + cmd.getName() + "_" + Integer.toHexString(cmd.hashCode()), true));
        CommandScheduler.getInstance()
                .onCommandFinish(cmd -> Logger.recordOutput(
                        "Commands/" + cmd.getName() + "_" + Integer.toHexString(cmd.hashCode()), false));
        CommandScheduler.getInstance()
                .onCommandInterrupt(cmd -> Logger.recordOutput(
                        "Commands/" + cmd.getName() + "_" + Integer.toHexString(cmd.hashCode()), false));

        RobotController.setBrownoutVoltage(6.5);

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        // Aiming calculation warmup
        RobotState.getInstance().getHubAimingParameters();
        RobotState.getInstance().getShuffleAimingParameters();
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Switch main robot thread to high priority to improve loop timing
        if (DriverStation.isEnabled()) Threads.setCurrentThreadPriority(true, 2);
        else Threads.setCurrentThreadPriority(false, 0);
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 0);
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousExit() {}

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopExit() {}

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
