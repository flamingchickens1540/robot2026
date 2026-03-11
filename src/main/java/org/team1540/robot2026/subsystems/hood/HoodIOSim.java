package org.team1540.robot2026.subsystems.hood;

import static org.team1540.robot2026.subsystems.intake.IntakeConstants.*;
import static org.team1540.robot2026.subsystems.intake.IntakeConstants.PIVOT_MAX_ANGLE;
import static org.team1540.robot2026.subsystems.intake.IntakeConstants.PIVOT_MIN_ANGLE;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.team1540.robot2026.util.LoggedTunableNumber;

public class HoodIOSim implements HoodIO {
    private static final LoggedTunableNumber SIM_PIVOT_KP = new LoggedTunableNumber("hood/p");
    private static final LoggedTunableNumber SIM_PIVOT_KI = new LoggedTunableNumber("hood/i");
    private static final LoggedTunableNumber SIM_PIVOT_KD = new LoggedTunableNumber("hood/d");
    private static final LoggedTunableNumber SIM_PIVOT_KS = new LoggedTunableNumber("hood/d");
    private static final LoggedTunableNumber SIM_PIVOT_KG = new LoggedTunableNumber("hood/d");
    private static final LoggedTunableNumber SIM_PIVOT_KV = new LoggedTunableNumber("hood/d");

    private final SingleJointedArmSim motor = new SingleJointedArmSim(
            DCMotor.getFalcon500Foc(1),
            PIVOT_GEAR_RATIO,
            0.1,
            PIVOT_MOI_KGM2,
            PIVOT_MIN_ANGLE.getRadians(),
            PIVOT_MAX_ANGLE.getRadians(),
            true,
            PIVOT_MAX_ANGLE.getRadians());
    double appliedVolts = 0;
    boolean motorIsClosedLoop;
    private ArmFeedforward pivotFeedforward =
            new ArmFeedforward(SIM_PIVOT_KS.getAsDouble(), SIM_PIVOT_KG.getAsDouble(), SIM_PIVOT_KV.getAsDouble());
    private final ProfiledPIDController pivotController = new ProfiledPIDController(
            SIM_PIVOT_KP.getAsDouble(),
            SIM_PIVOT_KI.getAsDouble(),
            SIM_PIVOT_KD.getAsDouble(),
            new TrapezoidProfile.Constraints(PIVOT_CRUISE_VELOCITY_RPS, PIVOT_ACCELERATION_RPS2));

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        motor.setInputVoltage(appliedVolts);
        if (motorIsClosedLoop) {
            appliedVolts = pivotController.calculate(Units.radiansToRotations(motor.getAngleRads()))
                    + pivotFeedforward.calculate(
                            Units.rotationsToRadians(pivotController.getSetpoint().position),
                            Units.rotationsToRadians(pivotController.getSetpoint().velocity));
        }
        motor.update(appliedVolts);

        inputs.motorConnected = true;
        inputs.velocityRPS = motor.getVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.supplyCurrentAmps = motor.getCurrentDrawAmps();
        inputs.statorCurrentAmps = motor.getCurrentDrawAmps();
        inputs.tempCelsius = 0.0;
    }

    @Override
    public void setVoltage(double volts) {
        motorIsClosedLoop = false;
        appliedVolts = volts;
    }

    @Override
    public void setSetpoint(Rotation2d position) {
        if (!motorIsClosedLoop) {
            pivotController.reset(
                    Units.radiansToRotations(motor.getAngleRads()),
                    Units.radiansToRotations(motor.getVelocityRadPerSec()));
        }
        motorIsClosedLoop = true;
        pivotController.setGoal(new TrapezoidProfile.State(position.getRotations(), 0));
    }

    @Override
    public void resetPosition(Rotation2d position) {
        pivotController.reset(position.getRotations());
    }

    @Override
    public void configPID(double kP, double kI, double kD, double kS, double kV, double kG) {
        pivotController.setP(kP);
        pivotController.setI(kI);
        pivotController.setD(kD);
        pivotFeedforward.setKs(kS);
        pivotFeedforward.setKv(kV);
        pivotFeedforward.setKg(kG);
    }
}
