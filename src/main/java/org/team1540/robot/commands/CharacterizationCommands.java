package org.team1540.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot.util.LoggedTunableNumber;
import org.team1540.robot.util.math.PolynomialRegression;

public class CharacterizationCommands {
    private static final double START_DELAY_SECS = 1.0;

    private static final LoggedTunableNumber ffRampVoltsPerSec =
            new LoggedTunableNumber("Characterization/Feedforward/RampVoltsPerSec", 0.1);

    public static Command feedforward(
            DoubleConsumer voltageConsumer, DoubleSupplier velocitySupplier, Subsystem... requirements) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        var command = Commands.sequence(
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),
                Commands.run(() -> voltageConsumer.accept(0.0)).withTimeout(START_DELAY_SECS),
                Commands.runOnce(timer::restart),
                Commands.run(() -> {
                            double voltage = timer.get() * ffRampVoltsPerSec.get();
                            voltageConsumer.accept(voltage);
                            velocitySamples.add(velocitySupplier.getAsDouble());
                            voltageSamples.add(voltage);
                        })
                        .finallyDo(() -> {
                            if (velocitySamples.isEmpty() || voltageSamples.isEmpty()) {
                                return;
                            }

                            PolynomialRegression regression = new PolynomialRegression(
                                    velocitySamples.stream()
                                            .mapToDouble(Double::doubleValue)
                                            .toArray(),
                                    voltageSamples.stream()
                                            .mapToDouble(Double::doubleValue)
                                            .toArray(),
                                    1);

                            System.out.println("********** Feedforward Characterization Results **********");
                            System.out.println("\tCount=" + velocitySamples.size());
                            System.out.printf("\tR2=%.5f%n", regression.R2());
                            System.out.printf("\tkS=%.5f%n", regression.beta(0));
                            System.out.printf("\tkV=%.5f%n", regression.beta(1));

                            Logger.recordOutput("Characterization/Feedforward/SampleCount", velocitySamples.size());
                            Logger.recordOutput("Characterization/Feedforward/R2", regression.R2());
                            Logger.recordOutput("Characterization/Feedforward/kS", regression.beta(0));
                            Logger.recordOutput("Characterization/Feedforward/kV", regression.beta(1));
                        }));
        command.addRequirements(requirements);
        return command;
    }
}
