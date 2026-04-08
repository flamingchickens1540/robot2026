package org.team1540.robot2026.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
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
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.SimState;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim spinSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), 0.001, SPIN_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(2));

    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
            DCMotor.getKrakenX44Foc(1),
            PIVOT_GEAR_RATIO,
            0.1,
            PIVOT_MOI_KGM2,
            PIVOT_MAX_ANGLE.getRadians(),
            -PIVOT_MIN_ANGLE.getRadians(),
            true,
            -PIVOT_MIN_ANGLE.getRadians());

    private double spinRequestedVolts = 0.0;
    private double spinAppliedVolts = 0.0;

    private double pivotRequestedVolts = 0.0;
    private double pivotAppliedVolts = 0.0;

    private final ProfiledPIDController pivotController = new ProfiledPIDController(
            PIVOT_KP,
            PIVOT_KI,
            PIVOT_KD,
            new TrapezoidProfile.Constraints(PIVOT_CRUISE_VELOCITY_RPS, PIVOT_ACCELERATION_RPS2));
    private final ArmFeedforward pivotFeedforward =
            new ArmFeedforward(PIVOT_KS, PIVOT_KG, 1 / Units.rotationsToRadians(1 / PIVOT_KV));
    private boolean isPivotClosedLoop;

    public IntakeIOSim() {
        pivotController.reset(PIVOT_MAX_ANGLE.getRotations());
        SimState.getInstance().addCurrentDraw(spinSim::getCurrentDrawAmps, () -> spinAppliedVolts);
        SimState.getInstance().addCurrentDraw(pivotSim::getCurrentDrawAmps, () -> pivotAppliedVolts);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        if (isPivotClosedLoop) {
            pivotRequestedVolts = pivotController.calculate(Units.radiansToRotations(pivotSim.getAngleRads()))
                    + pivotFeedforward.calculate(
                            Units.rotationsToRadians(pivotController.getSetpoint().position),
                            Units.rotationsToRadians(pivotController.getSetpoint().velocity));
        }
        pivotAppliedVolts =
                SimulatedBattery.clamp(Volts.of(pivotRequestedVolts)).in(Volts);
        spinAppliedVolts = SimulatedBattery.clamp(Volts.of(spinRequestedVolts)).in(Volts);

        spinSim.setInputVoltage(spinAppliedVolts);
        pivotSim.setInputVoltage(pivotAppliedVolts);
        spinSim.update(Constants.LOOP_PERIOD_SECS);
        pivotSim.update(Constants.LOOP_PERIOD_SECS);

        inputs.spinMotorAppliedVolts = new double[] {spinAppliedVolts};
        inputs.spinMotorVelocityRPS = new double[] {spinSim.getAngularVelocityRPM() / 60.0};
        inputs.spinStatorCurrentAmps = new double[] {spinSim.getCurrentDrawAmps()};
        inputs.spinSupplyCurrentAmps = new double[] {
            spinSim.getCurrentDrawAmps()
                    * spinAppliedVolts
                    / SimulatedBattery.getBatteryVoltage().in(Volts)
        };

        inputs.pivotMotorAppliedVolts = -pivotAppliedVolts;
        inputs.pivotPosition = Rotation2d.fromRadians(-pivotSim.getAngleRads());
        inputs.pivotMotorVelocityRPS = Units.rotationsToRadians(pivotSim.getVelocityRadPerSec()) / 60.0;
        inputs.pivotStatorCurrentAmps = pivotSim.getCurrentDrawAmps();
        inputs.pivotSupplyCurrentAmps = pivotSim.getCurrentDrawAmps()
                * pivotAppliedVolts
                / SimulatedBattery.getBatteryVoltage().in(Volts);
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        spinRequestedVolts = voltage;
    }

    @Override
    public void setPivotVoltage(double voltage) {
        isPivotClosedLoop = false;
        pivotRequestedVolts = -voltage;
    }

    @Override
    public void setPivotSetpoint(Rotation2d pivotPosition) {
        if (!isPivotClosedLoop) {
            pivotController.reset(
                    Units.radiansToRotations(pivotSim.getAngleRads()),
                    Units.radiansToRotations(pivotSim.getVelocityRadPerSec()));
        }
        isPivotClosedLoop = true;
        pivotController.setGoal(new TrapezoidProfile.State(-pivotPosition.getRotations(), 0));
    }

    @Override
    public void setPivotPID(double kP, double kI, double kD) {
        pivotController.setPID(kP, kI, kD);
    }

    @Override
    public void setPivotFF(double kS, double kV, double kG) {
        pivotFeedforward.setKs(kS);
        pivotFeedforward.setKv(kV);
        pivotFeedforward.setKg(kG);
    }
}
