package org.team1540.robot2026.subsystems.Shooter;

import static org.team1540.robot2026.subsystems.Shooter.ShooterConstants.Flywheels.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.team1540.robot2026.Constants;

public class ShooterIOSim implements FlywheelsIO {
    private static final double SIM_KP = 0;
    private static final double SIM_KI = 0;
    private static final double SIM_KD = 0;
    private static final double SIM_KS = 0;
    private static final double SIM_KV = 0;
    private static final double SIM_KG = 0;

    private final FlywheelSim leftFlywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), SIM_MOI, GEAR_RATIO), DCMotor.getKrakenX60(1));
    private final FlywheelSim rightFlywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), SIM_MOI, GEAR_RATIO), DCMotor.getKrakenX60(1));

    private final PIDController rightController = new PIDController(SIM_KP, SIM_KI, SIM_KD);
    private final PIDController leftController = new PIDController(SIM_KP, SIM_KI, SIM_KD);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SIM_KS, SIM_KV);

    private boolean isClosedLoop;
    private double leftSetpointRPS;
    private double rightSetpointRPS;

    private double leftVolts = 0.0;
    private double rightVolts = 0.0;

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        if (isClosedLoop) {
            leftVolts = leftController.calculate(leftFlywheelSim.getAngularVelocityRPM() / 60, leftSetpointRPS)
                    + feedforward.calculate(leftSetpointRPS);
            rightVolts = rightController.calculate(rightFlywheelSim.getAngularVelocityRPM() / 60, rightSetpointRPS)
                    + feedforward.calculate(rightSetpointRPS);
        }

        leftFlywheelSim.setInputVoltage(leftVolts);
        rightFlywheelSim.setInputVoltage(rightVolts);
        leftFlywheelSim.update(Constants.LOOP_PERIOD_SECS);
        rightFlywheelSim.update(Constants.LOOP_PERIOD_SECS);

        inputs.leftVelocityRPM = leftFlywheelSim.getAngularVelocityRPM();
        inputs.leftAppliedVolts = leftVolts;
        inputs.leftSupplyCurrentAmps = leftFlywheelSim.getCurrentDrawAmps();

        inputs.rightVelocityRPM = rightFlywheelSim.getAngularVelocityRPM();
        inputs.rightAppliedVolts = rightVolts;
        inputs.rightSupplyCurrentAmps = rightFlywheelSim.getCurrentDrawAmps();
    }

    @Override
    public void setSpeeds(double leftRPM, double rightRPM) {
        isClosedLoop = true;
        leftController.reset();
        rightController.reset();
        leftSetpointRPS = leftRPM / 60;
        rightSetpointRPS = rightRPM / 60;
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        isClosedLoop = false;
        this.leftVolts = leftVolts;
        this.rightVolts = rightVolts;
    }

    public void configPID(double kP, double kI, double kD, double kV) {
        leftController.setPID(kP, kI, kD);
        rightController.setPID(kP, kI, kD);

        feedforward = new SimpleMotorFeedforward(KS, kV);
    }
}
