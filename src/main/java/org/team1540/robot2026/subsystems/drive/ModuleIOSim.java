package org.team1540.robot2026.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.team1540.robot2026.subsystems.drive.DrivetrainConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.team1540.robot2026.SimState;

public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation moduleSim;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private final PIDController drivePID;
    private final SimpleMotorFeedforward driveFF;
    private final PIDController turnPID;

    private Voltage driveAppliedVolts = Volts.zero();
    private Voltage turnAppliedVolts = Volts.zero();

    private boolean driveClosedLoop;
    private boolean driveTorqueControl;
    private boolean turnClosedLoop;

    private double driveFFCurrentAmps;

    private final DCMotor driveMotorModel;

    public ModuleIOSim(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
            SwerveModuleSimulation moduleSim) {
        this.moduleSim = moduleSim;
        this.driveMotor =
                moduleSim.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(constants.SlipCurrent));
        this.turnMotor = moduleSim.useGenericControllerForSteer().withCurrentLimit(Amps.of(120.0));

        drivePID = DRIVE_TORQUE_CONTROL ? new PIDController(35, 0.0, 0.0) : new PIDController(0.8, 0.0, 0.0);
        driveFF = DRIVE_TORQUE_CONTROL
                ? new SimpleMotorFeedforward(0.05098, 0.83092)
                : new SimpleMotorFeedforward(5.0, 0.0);
        turnPID = new PIDController(75.0, 0.0, 0.0);

        driveMotorModel = DCMotor.getKrakenX60Foc(1).withReduction(constants.DriveMotorGearRatio);

        turnPID.enableContinuousInput(-0.5, 0.5);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if (driveClosedLoop) {
            if (!driveTorqueControl) {
                driveAppliedVolts = Volts.of(
                        drivePID.calculate(moduleSim.getDriveWheelFinalSpeed().in(RotationsPerSecond))
                                + driveFF.calculate(drivePID.getSetpoint()));
            } else {
                double driveSpeedRPS = moduleSim.getDriveWheelFinalSpeed().in(RotationsPerSecond);
                double driveAppliedTorqueCurrentAmps = drivePID.calculate(driveSpeedRPS)
                        + driveFF.calculate(drivePID.getSetpoint())
                        + driveFFCurrentAmps;
                driveAppliedVolts = Volts.of(driveMotorModel.getVoltage(
                        driveMotorModel.getTorque(driveAppliedTorqueCurrentAmps),
                        Units.rotationsToRadians(driveSpeedRPS)));
            }
        }
        if (turnClosedLoop) {
            turnAppliedVolts = Volts.of(
                    turnPID.calculate(moduleSim.getSteerAbsoluteFacing().getRotations()));
        }

        driveAppliedVolts = SimulatedBattery.clamp(driveAppliedVolts);
        turnAppliedVolts = SimulatedBattery.clamp(turnAppliedVolts);

        driveMotor.requestVoltage(driveAppliedVolts);
        turnMotor.requestVoltage(turnAppliedVolts);

        inputs.driveConnected = true;
        inputs.drivePositionRads = moduleSim.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadPerSec = moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.driveAppliedVolts = driveAppliedVolts.in(Volts);
        inputs.driveSupplyCurrentAmps = moduleSim.getDriveMotorSupplyCurrent().in(Amps);
        inputs.driveStatorCurrentAmps = moduleSim.getDriveMotorStatorCurrent().in(Amps);
        inputs.driveTorqueCurrentAmps = moduleSim.getDriveMotorStatorCurrent().in(Amps);

        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        inputs.turnAbsolutePosition = moduleSim.getSteerAbsoluteFacing();
        inputs.turnPosition = inputs.turnAbsolutePosition;
        inputs.turnVelocityRadPerSec = moduleSim.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.turnAppliedVolts = turnAppliedVolts.in(Volts);
        inputs.turnSupplyCurrentAmps = moduleSim.getSteerMotorSupplyCurrent().in(Amps);
        inputs.turnStatorCurrentAmps = moduleSim.getSteerMotorStatorCurrent().in(Amps);

        inputs.odometryTimestamps = SimState.getInstance().getSimOdometryTimestamps();
        inputs.odometryDrivePositionsRads = Arrays.stream(moduleSim.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Radians))
                .toArray();
        inputs.odometryTurnPositions = moduleSim.getCachedSteerAbsolutePositions();
    }

    @Override
    public void setDriveVelocityVoltage(double velocityRadPerSec) {
        driveClosedLoop = true;
        drivePID.setSetpoint(Units.radiansToRotations(velocityRadPerSec));
    }

    @Override
    public void setDriveVelocityTorqueCurrent(double velocityRadPerSec, double ffCurrentAmps) {
        driveClosedLoop = true;
        driveTorqueControl = true;
        driveFFCurrentAmps = ffCurrentAmps;
        drivePID.setSetpoint(Units.radiansToRotations(velocityRadPerSec));
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveClosedLoop = false;
        driveAppliedVolts = Volts.of(MathUtil.clamp(volts, -12.0, 12.0));
        drivePID.reset();
    }

    @Override
    public void setTurnPosition(Rotation2d position) {
        turnClosedLoop = true;
        turnPID.setSetpoint(position.getRotations());
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnClosedLoop = false;
        turnAppliedVolts = Volts.of(MathUtil.clamp(volts, -12.0, 12.0));
        turnPID.reset();
    }
}
