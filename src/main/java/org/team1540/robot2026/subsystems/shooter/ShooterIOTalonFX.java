package org.team1540.robot2026.subsystems.shooter;

import static org.team1540.robot2026.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leader = new TalonFX(RIGHT_ID);
    private final TalonFX follower = new TalonFX(LEFT_ID);

    private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
    private final StatusSignal<Voltage> leaderVoltage = leader.getMotorVoltage();
    private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
    private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();
    private final StatusSignal<Temperature> leaderTemp = leader.getDeviceTemp();

    private final StatusSignal<AngularVelocity> followerVelocity = follower.getVelocity();
    private final StatusSignal<Voltage> followerVoltage = follower.getMotorVoltage();
    private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent();
    private final StatusSignal<Current> followerSupplyCurrent = follower.getSupplyCurrent();
    private final StatusSignal<Temperature> followerTemp = follower.getDeviceTemp();

    private final VelocityVoltage velocityCtrlReq =
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VoltageOut voltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

    public ShooterIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = KP;
        config.Slot0.kI = KI;
        config.Slot0.kD = KD;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;

        leader.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);
        follower.setControl(new Follower(RIGHT_ID, MotorAlignmentValue.Opposed));

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                leaderVelocity,
                leaderVoltage,
                leaderStatorCurrent,
                leaderSupplyCurrent,
                leaderTemp,
                followerVelocity,
                followerVoltage,
                followerStatorCurrent,
                followerSupplyCurrent,
                followerTemp);

        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leaderConnected = BaseStatusSignal.refreshAll(
                        leaderVelocity, leaderVoltage, leaderStatorCurrent, leaderSupplyCurrent, leaderTemp)
                .isOK();
        inputs.followerConnected = BaseStatusSignal.refreshAll(
                        followerVelocity, followerVoltage, followerStatorCurrent, followerSupplyCurrent, followerTemp)
                .isOK();
        inputs.velocityRPM =
                new double[] {leaderVelocity.getValueAsDouble() * 60, followerVelocity.getValueAsDouble() * 60};
        inputs.appliedVolts = new double[] {leaderVoltage.getValueAsDouble(), followerVoltage.getValueAsDouble()};
        inputs.statorCurrentAmps =
                new double[] {leaderStatorCurrent.getValueAsDouble(), followerStatorCurrent.getValueAsDouble()};
        inputs.supplyCurrentAmps =
                new double[] {leaderSupplyCurrent.getValueAsDouble(), followerSupplyCurrent.getValueAsDouble()};
        inputs.tempCelsius = new double[] {leaderTemp.getValueAsDouble(), followerTemp.getValueAsDouble()};
    }

    @Override
    public void runVelocity(double velocityRPM) {
        leader.setControl(velocityCtrlReq.withVelocity(velocityRPM / 60));
    }

    @Override
    public void setVoltage(double volts) {
        leader.setControl(voltageCtrlReq.withOutput(volts));
    }

    @Override
    public void configPID(double kP, double kI, double kD, double kS, double kV) {
        Slot0Configs pidConfigs = new Slot0Configs();
        leader.getConfigurator().refresh(pidConfigs);
        pidConfigs.kP = kP;
        pidConfigs.kI = kI;
        pidConfigs.kD = kD;
        pidConfigs.kV = kV;
        pidConfigs.kS = kS;
        leader.getConfigurator().apply(pidConfigs);
        follower.getConfigurator().apply(pidConfigs);
    }
}
