package org.team1540.robot2026.subsystems.panaxis;

import static org.team1540.robot2026.subsystems.panaxis.TurretConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.Constants;
import org.team1540.robot2026.util.LoggedTracer;
import org.team1540.robot2026.util.LoggedTunableNumber;

import java.util.function.DoubleSupplier;

public class Turret extends SubsystemBase {
    private boolean hasInstance = false;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", KD);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", KV);


    private final Alert motorDisconnectedAlert = new Alert("Turret motor disconnected", Alert.AlertType.kError);

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();




    private Rotation2d setpointRotation;

    private final Debouncer zeroedDebouncer = new Debouncer(0.25);
    private boolean atZero = false;

    private Turret(TurretIO turretIO) {
        if (hasInstance) throw new IllegalStateException("Instance of elevator already exists");
        hasInstance = true;
        this.io = turretIO;
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        if (RobotState.isDisabled()) stop();

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configFF(kS.get(), kV.get(), kG.get()), kS, kV, kG);

        // MechanismVisualizer.getInstance().setElevatorPosition(inputs.positionMeters[0]); TODO set up sim pos

        motorDisconnectedAlert.set(!inputs.connected || !inputs.mainEncoderConnected || !inputs.secondaryEncoderConnected);

        LoggedTracer.record("Turret");
    }

    public void stop() {
        io.setVoltage(0);
    }

    @AutoLogOutput(key = "Turret/AtSetpoint")
    public boolean isAtSetpoint() {
        return MathUtil.isNear(setpointRotation.getDegrees(), inputs.position.getDegrees(), POS_ERR_TOLERANCE_DEGREES);
    }

    @AutoLogOutput(key = "Turret/Setpoint")
    public Rotation2d getSetpoint() {
        return setpointRotation;
    }

    public void setSetpoint(Rotation2d position) {

      position.fromDegrees(MathUtil.clamp(position.getDegrees(), 0, MAX_TURRET_ROTATION));
        setpointRotation = position;
        io.setSetpoint(setpointRotation);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public Rotation2d getPosition() {
        return inputs.position;
    }

    public double getVelocity() {
        return inputs.velocityRadPerSec;
    }

    public void setBrakeMode(boolean isBrakeMode) {
        io.setBrakeMode(isBrakeMode);
    }

    public double calculateTurretAngleFromEncoderInputs(double mainEncoderPos, double secondaryEncoderPos) {
        // Inputs are integer readings from the encoders, so convert
        double e1 = (mainEncoderPos)*ENCODER_RANGE;
        double e2 = (secondaryEncoderPos)*ENCODER_RANGE;
        // Handle an edge case near 0 degrees where one or both encoder is 'below zero', that is, has
        // a large value
        double e1Rotations = -1;
        double e2Rotations = -1;
        // Now search through all possible rotation pairs to find the smallest error
        double minError = DrivenGearDiameter;
        double minE1Rotations = e1Rotations;
        while (e1Rotations < ROTATIONS_OF_GEAR_1_PER_DRIVEN_GEAR_ROTATION && e2Rotations < ROTATIONS_OF_GEAR_2_PER_DRIVEN_GEAR_ROTATION) {
            double e1Value = e1 + e1Rotations * ENCODER_RANGE;
            double e2Value = e2 + (e2Rotations) * ENCODER_RANGE;
            double e1Distance = e1Value * PlanetaryGear1InchPerBit;
            double e2Distance = e2Value * PlanetaryGear2InchPerBit;
            double error = Math.abs(e1Distance - e2Distance);
            if (error < minError) {
                minError = error;
                minE1Rotations = e1Rotations;
            }

            if (e1Distance < e2Distance) {
                e1Rotations += 1;
            } else {
                e2Rotations += 1;
            }
        }
        //Now the best answer is given by minE1Rotations, so compute the distance gear 1
        //has moved and convert to the corresponding driven gear movement.Note that
        //the distance gear 1 has moved (at the pitch circle)is equal to the distance
        //the driven gear has moved at its pitch circle
        double e1Count = e1 + (minE1Rotations) * ENCODER_RANGE;
        double e1Distance = e1Count * PlanetaryGear1InchPerBit;
        double turretAngle = e1Distance * DrivenGearDegreePerInch;
        return turretAngle;
    }

    public Command commandToSetpoint(DoubleSupplier rotationDegrees) {
        return Commands.runOnce(() -> setSetpoint(rotationDegrees.getAsDouble()),this
        );
    }

    public Command zeroCommand() {
        return Commands.run(()-> setSetpoint(calculateTurretAngleFromEncoderInputs(
                Units.rotationsToDegrees(mainCancoder.getAbsolutePosition().getValueAsDouble()),
                Units.rotationsToDegrees(secondaryCancoder.getAbsolutePosition().getValueAsDouble()))));
    }

    public static Turret createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real turret on simulated robot", false);
        }
        return new Turret(new TurretIOTalonFX());
    }

    public static Turret createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated turret on real robot", false);
        }
        return new Turret(new TurretIOSim());
    }

    public static Turret createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy turret on real robot", false);
        }
        return new Turret(new TurretIO() {});
    }
}
