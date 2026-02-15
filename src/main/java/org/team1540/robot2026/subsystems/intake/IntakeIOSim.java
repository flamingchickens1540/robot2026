package org.team1540.robot2026.subsystems.intake;

import static org.team1540.robot2026.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.team1540.robot2026.Constants;

public class IntakeIOSim implements IntakeIO {
    private static final double SIM_PIVOT_KS = 0.0;
    private static final double SIM_PIVOT_KV = 0.0;
    private static final double SIM_PIVOT_KG = 0.0;
    private static final double SIM_PIVOT_KP = 0.0;
    private static final double SIM_PIVOT_KI = 0.0;
    private static final double SIM_PIVOT_KD = 0.0;

    private final DCMotorSim spinSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(1), 0.001, SPIN_GEAR_RATIO),
            DCMotor.getFalcon500Foc(1));

    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
            DCMotor.getFalcon500Foc(1),
            PIVOT_GEAR_RATIO,
            0.1,
            PIVOT_MOI,
            PIVOT_MIN_ANGLE.getRadians(),
            PIVOT_MAX_ANGLE.getRadians(),
            true,
            PIVOT_MAX_ANGLE.getRadians());

    private double spinAppliedVolts = 0.0;
    private double pivotAppliedVolts = 0.0;

    private final ProfiledPIDController pivotController = new ProfiledPIDController(
            SIM_PIVOT_KP,
            SIM_PIVOT_KI,
            SIM_PIVOT_KD,
            new TrapezoidProfile.Constraints(PIVOT_CRUISE_VELOCITY_RPS, PIVOT_ACCELERATION_RPS2));

    private ArmFeedforward pivotFeedforward = new ArmFeedforward(SIM_PIVOT_KS, SIM_PIVOT_KG, SIM_PIVOT_KV);
    private boolean isPivotClosedLoop;

    public IntakeIOSim() {
        pivotController.reset(PIVOT_MAX_ANGLE.getRotations());
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        if (isPivotClosedLoop) {
            pivotAppliedVolts = pivotController.calculate(Units.radiansToRotations(pivotSim.getAngleRads()))
                    + pivotFeedforward.calculate(
                            Units.rotationsToRadians(pivotController.getSetpoint().position),
                            Units.rotationsToRadians(pivotController.getSetpoint().velocity));
        }
        spinSim.setInputVoltage(spinAppliedVolts);
        pivotSim.setInputVoltage(pivotAppliedVolts);
        spinSim.update(Constants.LOOP_PERIOD_SECS);
        pivotSim.update(Constants.LOOP_PERIOD_SECS);

        inputs.spinMotorAppliedVolts = spinAppliedVolts;
        inputs.spinMotorVelocityRPS = spinSim.getAngularVelocityRPM() / 60.0;
        inputs.spinStatorCurrentAmps = spinSim.getCurrentDrawAmps();
        inputs.spinSupplyCurrentAmps = spinSim.getCurrentDrawAmps();

        inputs.pivotMotorAppliedVolts = pivotAppliedVolts;
        inputs.pivotPosition = Rotation2d.fromRadians(pivotSim.getAngleRads());
        inputs.pivotMotorVelocityRPS = Units.rotationsToRadians(pivotSim.getVelocityRadPerSec()) / 60.0;
        inputs.pivotStatorCurrentAmps = pivotSim.getCurrentDrawAmps();
        inputs.pivotSupplyCurrentAmps = pivotSim.getCurrentDrawAmps();
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        spinAppliedVolts = voltage;
    }

    @Override
    public void setPivotVoltage(double voltage) {
        isPivotClosedLoop = false;
        pivotAppliedVolts = voltage;
    }

    @Override
    public void setPivotPID(double kP, double kI, double kD) {
        pivotController.setPID(kP, kI, kD);
    }

    @Override
    public void setPivotFF(double kS, double kV, double kG) {
        pivotFeedforward = new ArmFeedforward(kS, kG, kV);
    }

    @Override
    public void setPivotSetpoint(Rotation2d pivotPosition) {
        if (!isPivotClosedLoop) {
            pivotController.reset(
                    Units.radiansToRotations(pivotSim.getAngleRads()),
                    Units.radiansToRotations(pivotSim.getVelocityRadPerSec()));
        }
        isPivotClosedLoop = true;
        pivotController.setGoal(new TrapezoidProfile.State(pivotPosition.getRotations(), 0));
    }
}
