package org.team1540.robot2026.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team1540.robot2026.autos.AutoConfigurator.StartingSide;

public class AutoSelector {
    private static final AutoRoutineData defaultAuto =
            new AutoRoutineData("None", StartingSide.NONE, Optional.empty(), List.of(), List.of(), Commands.none());

    private final LoggedDashboardChooser<String> chooser = new LoggedDashboardChooser<>("Auto/Auto Selector");
    private final HashMap<String, Supplier<AutoRoutineData>> autoRoutines = new HashMap<>();

    private AutoRoutineData selectedAuto;
    private AutoRoutineData configuredAuto;

    public AutoSelector(AutoConfigurator configurator) {
        chooser.addDefaultOption("None", "None");
        selectedAuto = defaultAuto;
        chooser.onChange(autoName -> selectedAuto =
                autoRoutines.getOrDefault(autoName, () -> defaultAuto).get());

        addAuto("Configured Auto", configurator::getSelectedAuto);
        configurator.addChangeListener(auto -> {
            configuredAuto = configurator.getSelectedAuto();
            if (chooser.get() != null && chooser.get().equals("Configured Auto")) selectedAuto = configuredAuto;
        });
    }

    public void addAuto(String name, Supplier<AutoRoutineData> auto) {
        chooser.addOption(name, name);
        autoRoutines.put(name, auto);
    }

    public void addCmd(String name, Supplier<Command> cmd) {
        addAuto(
                name,
                () -> new AutoRoutineData(name, StartingSide.NONE, Optional.empty(), List.of(), List.of(), cmd.get()));
    }

    public AutoRoutineData getSelectedAuto() {
        return selectedAuto;
    }
}
