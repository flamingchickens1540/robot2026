package org.team1540.robot2026.util.sim;

import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public class CustomRebuiltArena extends Arena2026Rebuilt {
    public CustomRebuiltArena() {
        this(false);
    }

    public CustomRebuiltArena(boolean addBumpCollider) {
        super(addBumpCollider);

        // Clear simulations so we can add fixed hub simulations
        customSimulations.clear();

        blueHub = new CustomRebuiltHub(this, true);
        addCustomSimulation(blueHub);

        redHub = new CustomRebuiltHub(this, false);
        addCustomSimulation(redHub);

        // Add outposts back
        addCustomSimulation(blueOutpost);
        addCustomSimulation(redOutpost);
    }
}
