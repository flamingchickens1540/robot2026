package org.team1540.robot2026;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static org.team1540.robot2026.subsystems.turret.TurretConstants.ROBOT_TO_TURRET_2D;
import static org.team1540.robot2026.subsystems.turret.TurretConstants.ROBOT_TO_TURRET_3D;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.generated.TunerConstants;
import org.team1540.robot2026.subsystems.drive.DrivetrainConstants;

public class SimState {
    private static SimState instance = null;

    public static SimState getInstance() {
        if (instance == null) instance = new SimState();
        return instance;
    }

    private final SwerveDriveSimulation driveSim;

    private SimState() {
        if (Constants.CURRENT_MODE != Constants.Mode.SIM)
            throw new IllegalStateException("SimState should only be used in simulation");

        Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
        arena.setEfficiencyMode(true);
        SimulatedArena.overrideInstance(arena);
        SimulatedArena.getInstance().resetFieldForAuto();

        var simConfig = DriveTrainSimulationConfig.Default()
                .withRobotMass(Kilograms.of(Constants.ROBOT_MASS_KG))
                .withCustomModuleTranslations(DrivetrainConstants.getModuleTranslations())
                .withBumperSize(
                        Meters.of(Constants.BUMPER_LENGTH_X_METERS), Meters.of(Constants.BUMPER_LENGTH_Y_METERS))
                .withGyro(() -> new GyroSimulation(0.12 / 120, 0.02))
                .withSwerveModule(() -> new SwerveModuleSimulation(new SwerveModuleSimulationConfig(
                        DCMotor.getKrakenX60Foc(1),
                        DCMotor.getFalcon500(1),
                        TunerConstants.FrontLeft.DriveMotorGearRatio,
                        TunerConstants.FrontLeft.SteerMotorGearRatio,
                        Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                        Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                        Meters.of(TunerConstants.FrontLeft.WheelRadius),
                        KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                        DrivetrainConstants.WHEEL_COF)));
        driveSim = new SwerveDriveSimulation(simConfig, Pose2d.kZero);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
        AutoLogOutputManager.addObject(this);
    }

    public void update() {
        Logger.recordOutput(
                "SimState/Fuel",
                SimulatedArena.getInstance().getGamePiecesPosesByType("Fuel").toArray(new Pose3d[0]));

        SimulatedArena.getInstance().simulationPeriodic();
    }

    public void addCurrentDraw(DoubleSupplier statorCurrentAmps, DoubleSupplier appliedVoltage) {
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(statorCurrentAmps.getAsDouble()
                * appliedVoltage.getAsDouble()
                / SimulatedBattery.getBatteryVoltage().in(Volts)));
    }

    public SwerveDriveSimulation getDriveSim() {
        return driveSim;
    }

    @AutoLogOutput(key = "SimState/RobotPose")
    public Pose2d getSimulatedRobotPose() {
        return driveSim.getSimulatedDriveTrainPose();
    }

    @AutoLogOutput(key = "SimState/TurretPose")
    public Pose2d getSimulatedTurretPose() {
        return getSimulatedRobotPose()
                .transformBy(new Transform2d(
                        ROBOT_TO_TURRET_2D.getTranslation(),
                        RobotState.getInstance().getTurretAngle()));
    }

    @AutoLogOutput(key = "SimState/TurretPose3d")
    public Pose3d getSimulatedTurretPose3d() {
        return new Pose3d(getSimulatedRobotPose())
                .transformBy(new Transform3d(
                        ROBOT_TO_TURRET_3D.getTranslation(),
                        new Rotation3d(
                                0, 0, RobotState.getInstance().getTurretAngle().getRadians())));
    }

    public void resetSimPose(Pose2d pose) {
        driveSim.setSimulationWorldPose(pose);
    }

    public double[] getSimOdometryTimestamps() {
        double[] odometryTimestamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimestamps.length; i++) {
            odometryTimestamps[i] = Timer.getFPGATimestamp()
                    - 0.02
                    + i * SimulatedArena.getSimulationDt().in(Seconds);
        }
        return odometryTimestamps;
    }
}
