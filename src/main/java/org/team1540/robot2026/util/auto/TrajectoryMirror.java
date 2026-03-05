package org.team1540.robot2026.util.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import org.team1540.robot2026.FieldConstants;

import java.util.Arrays;
import java.util.List;

public class TrajectoryMirror {
    public static AutoTrajectory apply(AutoTrajectory traj, AutoRoutine routine) {
        Trajectory<SwerveSample> rawTraj = traj.getRawTrajectory();
        List<SwerveSample> samples = rawTraj.samples().stream().map(sample -> new SwerveSample(
                sample.t,
                sample.x,
                FieldConstants.fieldWidth - sample.y,
                -sample.heading,
                sample.vx,
                -sample.vy,
                -sample.omega,
                sample.ax,
                -sample.ay,
                -sample.alpha,
                sample.moduleForcesX(),
                Arrays.stream(sample.moduleForcesY()).map(f -> -f).toArray())).toList();
        Trajectory<SwerveSample> mirroredTraj = new Trajectory<>(rawTraj.name() + "_Mirrored", samples, rawTraj.splits(), rawTraj.events());
        return routine.trajectory(mirroredTraj);
    }
}
