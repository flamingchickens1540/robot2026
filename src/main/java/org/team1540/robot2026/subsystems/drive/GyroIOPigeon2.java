package org.team1540.robot2026.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static org.team1540.robot2026.subsystems.drive.DrivetrainConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import java.util.Objects;
import java.util.Queue;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.generated.TunerConstants;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, CAN_BUS);

    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<Angle> pitch = pigeon.getPitch();
    private final StatusSignal<Angle> roll = pigeon.getRoll();

    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
    private final StatusSignal<AngularVelocity> pitchVelocity = pigeon.getAngularVelocityYWorld();
    private final StatusSignal<AngularVelocity> rollVelocity = pigeon.getAngularVelocityXWorld();

    private final StatusSignal<LinearAcceleration> xAcceleration = pigeon.getAccelerationX();
    private final StatusSignal<LinearAcceleration> yAcceleration = pigeon.getAccelerationY();
    private final StatusSignal<LinearAcceleration> zAcceleration = pigeon.getAccelerationZ();

    private final StatusSignal<Double> xGravity = pigeon.getGravityVectorX();
    private final StatusSignal<Double> yGravity = pigeon.getGravityVectorY();
    private final StatusSignal<Double> zGravity = pigeon.getGravityVectorZ();

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOPigeon2() {
        pigeon.getConfigurator()
                .apply(Objects.requireNonNullElse(
                        TunerConstants.DrivetrainConstants.Pigeon2Configs, new Pigeon2Configuration()));
        pigeon.getConfigurator().setYaw(0.0);

        yaw.setUpdateFrequency(ODOMETRY_FREQUENCY);
        BaseStatusSignal.setUpdateFrequencyForAll(
                1.0 / Constants.LOOP_PERIOD_SECS,
                pitch,
                roll,
                yawVelocity,
                pitchVelocity,
                rollVelocity,
                xAcceleration,
                yAcceleration,
                zAcceleration,
                xGravity,
                yGravity,
                zGravity);
        pigeon.optimizeBusUtilization();

        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance().registerSignal(pigeon.getYaw());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                        yaw,
                        pitch,
                        roll,
                        yawVelocity,
                        pitchVelocity,
                        rollVelocity,
                        xAcceleration,
                        yAcceleration,
                        zAcceleration,
                        xGravity,
                        yGravity,
                        zGravity)
                .equals(StatusCode.OK);

        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.pitchPosition = Rotation2d.fromDegrees(pitch.getValueAsDouble());
        inputs.rollPosition = Rotation2d.fromDegrees(roll.getValueAsDouble());

        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(pitchVelocity.getValueAsDouble());
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(rollVelocity.getValueAsDouble());

        inputs.xAccelMPS2 = xAcceleration.getValue().in(MetersPerSecondPerSecond);
        inputs.yAccelMPS2 = yAcceleration.getValue().in(MetersPerSecondPerSecond);
        inputs.zAccelMPS2 = zAcceleration.getValue().in(MetersPerSecondPerSecond);

        inputs.xGravity = xGravity.getValueAsDouble();
        inputs.yGravity = yGravity.getValueAsDouble();
        inputs.zGravity = zGravity.getValueAsDouble();

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
                yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
