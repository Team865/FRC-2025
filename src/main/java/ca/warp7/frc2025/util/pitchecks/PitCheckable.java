package ca.warp7.frc2025.util.pitchecks;

import edu.wpi.first.wpilibj2.command.Command;

public interface PitCheckable {
    /**
     * Returns a command which runs a sequence of checks defined by that subsystem
     */
    Command check();
}
