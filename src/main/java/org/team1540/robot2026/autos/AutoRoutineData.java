package org.team1540.robot2026.autos;

import static org.team1540.robot2026.autos.AutoConfigurator.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import java.util.Optional;

public record AutoRoutineData(
        String name,
        StartingSide startingSide,
        Optional<Pose2d> startingPose,
        List<SweepPath> sweeps,
        List<Pose2d> trajectoryPoints,
        Command command) {}
