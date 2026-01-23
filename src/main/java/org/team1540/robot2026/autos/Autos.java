package org.team1540.robot2026.autos;

import choreo.auto.AutoFactory;
import org.team1540.robot2026.RobotState;
import org.team1540.robot2026.subsystems.drive.Drivetrain;
import org.team1540.robot2026.util.AllianceFlipUtil;

public class Autos {
    private final AutoFactory autoFactory;

    private final Drivetrain drivetrain;

    public Autos(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        autoFactory = new AutoFactory(
                RobotState.getInstance()::getEstimatedPose,
                RobotState.getInstance()::resetPose,
                drivetrain::followTrajectory,
                true,
                drivetrain,
                (trajectory, starting) -> {
                    if (starting) {
                        RobotState.getInstance()
                                .setActiveTrajectory(
                                        (AllianceFlipUtil.shouldFlip() ? trajectory.flipped() : trajectory).getPoses());
                    } else {
                        RobotState.getInstance().clearActiveTrajectory();
                    }
                });
    }
}
