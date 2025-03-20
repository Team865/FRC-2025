package ca.warp7.frc2025.util.pitchecks;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PitChecker {
    private static PitCheckable[] checkables = new PitCheckable[0];

    public static void registerCheck(PitCheckable checkable) {
        PitCheckable[] newCheckables = new PitCheckable[checkables.length + 1];
        System.arraycopy(checkables, 0, newCheckables, 0, checkables.length);
        newCheckables[newCheckables.length - 1] = checkable;
        checkables = newCheckables;
    }

    public static Command runChecks() {
        Command command = Commands.sequence();

        for (var checkable : checkables) {
            command = command.andThen(checkable.check());
        }

        return command;
    }
}
