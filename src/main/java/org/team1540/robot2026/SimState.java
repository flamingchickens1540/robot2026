package org.team1540.robot2026;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static org.team1540.robot2026.subsystems.turret.TurretConstants.ROBOT_TO_TURRET_2D;
import static org.team1540.robot2026.subsystems.turret.TurretConstants.ROBOT_TO_TURRET_3D;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Random;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.team1540.robot2026.generated.TunerConstants;
import org.team1540.robot2026.subsystems.drive.DrivetrainConstants;
import org.team1540.robot2026.subsystems.hood.HoodConstants;
import org.team1540.robot2026.subsystems.intake.IntakeConstants;
import org.team1540.robot2026.util.AllianceFlipUtil;
import org.team1540.robot2026.util.sim.CustomRebuiltArena;

public class SimState {
    private static final Rotation2d INTAKE_EXTENSION_THRESH = Rotation2d.fromDegrees(-30);
    private static final double INTAKE_VOLTAGE_THRESH = 2.0;

    private static final double SPINDEXER_VOLTAGE_THRESH = 2.0;
    private static final double FEEDER_VOLTAGE_THRESH = 2.0;

    private static final double SHOT_ORIGIN_HEIGHT_METERS = ROBOT_TO_TURRET_3D.getZ() + Units.inchesToMeters(5.875);

    private static final double BPS = 8.0;
    private static final double BPS_STDDEV = 2.54;

    private static SimState instance = null;

    public static SimState getInstance() {
        if (instance == null) instance = new SimState();
        return instance;
    }

    private final SwerveDriveSimulation driveSim;
    private final IntakeSimulation intakeSim;

    private Rotation2d hoodAngle = HoodConstants.MIN_ANGLE;

    private Rotation2d intakeAngle = IntakeConstants.PIVOT_MAX_ANGLE;
    private double intakeVoltage = 0.0;

    private double spinVoltage = 0.0;
    private double feeder1Voltage = 0.0;
    private double feeder2Voltage = 0.0;

    private double shooterRPM = 0.0;

    private final Random rng = new Random(); // RNG for bps

    private final LoggedNetworkBoolean efficiencyMode =
            new LoggedNetworkBoolean("SmartDashboard/Sim/Performance Mode", false);

    private SimState() {
        if (Constants.CURRENT_MODE != Constants.Mode.SIM)
            throw new IllegalStateException("SimState should only be used in simulation");

        CustomRebuiltArena arena = new CustomRebuiltArena(false);
        arena.setEfficiencyMode(efficiencyMode.get());
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
        driveSim = new SwerveDriveSimulation(
                simConfig, new Pose2d(3, FieldConstants.LinesHorizontal.center, Rotation2d.kZero));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);

        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                driveSim,
                Meters.of(IntakeConstants.WIDTH_METERS),
                Meters.of(IntakeConstants.MAX_EXTENSION_METERS).minus(Inches.of(4.125)),
                IntakeSimulation.IntakeSide.FRONT,
                45);
        intakeSim.setCustomIntakeCondition(gamePiece -> isIntakeRunning());

        new Trigger(efficiencyMode)
                .onChange(Commands.runOnce(() -> arena.setEfficiencyMode(efficiencyMode.get()))
                        .ignoringDisable(true));

        new Trigger(this::isIntakeExtended)
                .onTrue(Commands.runOnce(intakeSim::startIntake))
                .onFalse(Commands.runOnce(intakeSim::stopIntake));
        new Trigger(this::isSpindexerRunning)
                .whileTrue(Commands.defer(
                                () -> Commands.waitSeconds(1.0 / rng.nextGaussian(BPS, BPS_STDDEV))
                                        .andThen(this::shootFuel),
                                Set.of())
                        .repeatedly());
        AutoLogOutputManager.addObject(this);
    }

    public void update() {
        Logger.recordOutput(
                "SimState/Fuel",
                SimulatedArena.getInstance().getGamePiecesPosesByType("Fuel").toArray(new Pose3d[0]));
        Logger.recordOutput("SimState/FuelInHopper", intakeSim.getGamePiecesAmount());
        SimulatedArena.getInstance().simulationPeriodic();
    }

    public void resetForAuto(Pose2d initialBluePose) {
        SimulatedArena.getInstance().resetFieldForAuto();
        intakeSim.setGamePiecesCount(8);
        if (initialBluePose != null) RobotState.getInstance().resetPose(AllianceFlipUtil.apply(initialBluePose));
    }

    public void addCurrentDraw(DoubleSupplier statorCurrentAmps, DoubleSupplier appliedVoltage) {
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(statorCurrentAmps.getAsDouble()
                * appliedVoltage.getAsDouble()
                / SimulatedBattery.getBatteryVoltage().in(Volts)));
    }

    public void addHoodData(Rotation2d angle) {
        hoodAngle = angle;
    }

    public void addIntakeData(Rotation2d angle, double voltage) {
        intakeAngle = angle;
        intakeVoltage = voltage;
    }

    public void addSpindexerData(double spinVoltage, double feeder1Voltage, double feeder2Voltage) {
        this.spinVoltage = spinVoltage;
        this.feeder1Voltage = feeder1Voltage;
        this.feeder2Voltage = feeder2Voltage;
    }

    public void addShooterData(double leftRPM, double rightRPM) {
        shooterRPM = (leftRPM + rightRPM) / 2.0;
    }

    @AutoLogOutput(key = "SimState/IntakeExtended")
    public boolean isIntakeExtended() {
        return intakeAngle.getDegrees() >= INTAKE_EXTENSION_THRESH.getDegrees();
    }

    @AutoLogOutput(key = "SimState/IntakeRunning")
    public boolean isIntakeRunning() {
        return isIntakeExtended() && intakeVoltage >= INTAKE_VOLTAGE_THRESH;
    }

    @AutoLogOutput(key = "SimState/SpindexerRunning")
    public boolean isSpindexerRunning() {
        return spinVoltage >= SPINDEXER_VOLTAGE_THRESH
                && feeder1Voltage >= FEEDER_VOLTAGE_THRESH
                && feeder2Voltage >= FEEDER_VOLTAGE_THRESH;
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

    private double calculateShotVelocityMPS(double shooterRPM) {
        return 0.00116376 * shooterRPM * Math.log(-0.00574055 * shooterRPM + 33.41442) + 0.254;
    }

    private void shootFuel() {
        if (!intakeSim.obtainGamePieceFromIntake()) return;

        Pose2d robotPose = getSimulatedRobotPose();
        ChassisSpeeds robotSpeeds = driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
        Translation2d turretSpeedFromRobotRotation = new Translation2d(
                robotSpeeds.omegaRadiansPerSecond
                        * ROBOT_TO_TURRET_2D.getTranslation().getNorm(),
                robotPose.getRotation().rotateBy(Rotation2d.kCW_90deg));
        ChassisSpeeds turretSpeeds = robotSpeeds.plus(new ChassisSpeeds(
                turretSpeedFromRobotRotation.getX(),
                turretSpeedFromRobotRotation.getY(),
                RobotState.getInstance().getTurretVelocityRadPerSec()));

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new RebuiltFuelOnFly(
                        robotPose.transformBy(ROBOT_TO_TURRET_2D).getTranslation(),
                        Translation2d.kZero,
                        turretSpeeds,
                        RobotState.getInstance().getFieldRelativeTurretAngle(),
                        Meters.of(SHOT_ORIGIN_HEIGHT_METERS),
                        MetersPerSecond.of(calculateShotVelocityMPS(shooterRPM)),
                        Rotation2d.kCCW_90deg.minus(hoodAngle).getMeasure()));
    }
}
