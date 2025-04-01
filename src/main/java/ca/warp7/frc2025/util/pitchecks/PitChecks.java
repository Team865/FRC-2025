package ca.warp7.frc2025.util.pitchecks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PitChecks {
    public static Command rampCheck(
            double rampRate,
            double max,
            double tolerance,
            Supplier<Double> actualValue,
            Consumer<Double> setValue,
            double timeout,
            String name) {

        SlewRateLimiter limter = new SlewRateLimiter(rampRate);

        double startingValue = actualValue.get();

        Command ramp = Commands.run(() -> {
                    double position = limter.calculate(max);
                    setValue.accept(position);
                })
                .withTimeout(timeout * 2);
        Command reverseRamp = Commands.run(() -> {
                    double position = limter.calculate(startingValue);
                    setValue.accept(position);
                })
                .withTimeout(timeout * 2);
        Command check = Commands.runOnce(() -> {
            if (MathUtil.isNear(max, actualValue.get(), tolerance)) {
                Logger.recordOutput("Pitchecks/" + name + "/rampForward", true);
            } else {
                Logger.recordOutput("Pitchecks/" + name + "/rampForward", false);
            }
        });
        Command reverseCheck = Commands.runOnce(() -> {
            if (MathUtil.isNear(max, actualValue.get(), tolerance)) {
                Logger.recordOutput("Pitchecks/" + name + "/rampReverse", true);
            } else {
                Logger.recordOutput("Pitchecks/" + name + "/rampReverse", false);
            }
        });

        return Commands.runOnce(() -> {
                    limter.reset(startingValue);
                    Logger.recordOutput("Pitchecks/" + name + "/rampForward", false);
                    Logger.recordOutput("Pitchecks/" + name + "/rampReverse", false);
                })
                .andThen(ramp)
                .andThen(check)
                .andThen(reverseRamp)
                .andThen(reverseCheck);
    }

    /**
     * Runs a pit check for reaching some target value (position, speed, etc.). If you are checking
     * multiple values, the order of those values in the arrays (expectedValues, tolerances,
     * measuredValues) must all be the same.
     *
     * @param expectedValues A function that returns your expected/target values as an array.
     * @param tolerance A function that returns your tolerances as an array.
     * @param measuredValues A function that returns your measured values as an array.
     * @param cmd The command to run the check on.
     * @param time How many seconds after the start of the check it will sample at. The whole sequence
     *     will run for 2x this time.
     * @param name The name of the check.
     */
    public static Command goalCheck(
            double[] targets,
            String[] setpointNames,
            double tolerance,
            double startingValue,
            Function<Double, Command> setGoal,
            Function<Double, Trigger> atGoal,
            double timeout,
            String name) {
        Supplier<Command>[] goToPoses = new Supplier[targets.length];

        Supplier<Command> goToStartingPose = () -> setGoal.apply(startingValue)
                .asProxy()
                .andThen(new WaitUntilCommand(atGoal.apply(startingValue)))
                .finallyDo(() -> {
                    if (atGoal.apply(startingValue).getAsBoolean()) {
                        Logger.recordOutput("Pitchecks/" + name + "/goToGoal/starting", true);
                    } else {
                        Logger.recordOutput("Pitchecks/" + name + "/goToGoal/starting", false);
                    }
                })
                .beforeStarting(() -> {
                    Logger.recordOutput("Pitchecks/" + name + "/goToGoal/starting", false);
                })
                .withTimeout(timeout * 2);

        for (int i = 0; i < targets.length; i++) {
            int index = i;
            double target = targets[i];
            goToPoses[i] = () -> setGoal.apply(target)
                    .asProxy()
                    .andThen(new WaitUntilCommand(atGoal.apply(target)))
                    .finallyDo(() -> {
                        Logger.recordOutput(
                                "Pitchecks/" + name + "/goToGoal/" + setpointNames[index],
                                atGoal.apply(target).getAsBoolean());
                    })
                    .beforeStarting(() -> {
                        // Logger.recordOutput(
                        //         "Pitchecks/" + name + "/goToGoal/" + String.valueOf(index) + "/goal", target);
                        Logger.recordOutput("Pitchecks/" + name + "/goToGoal/" + setpointNames[index], false);
                    })
                    .withTimeout(timeout * 2);
        }

        Command checkCommand = Commands.sequence();

        for (var goToPose : goToPoses) {
            checkCommand = checkCommand.andThen(goToPose.get());
        }

        checkCommand = checkCommand.andThen(goToStartingPose.get());

        for (var goToPose : goToPoses) {
            checkCommand = checkCommand.andThen(goToPose.get()).andThen(goToStartingPose.get());
        }

        return checkCommand;
    }

    /**
     * Runs a pit check for reaching some target value (position, speed, etc.). If you are checking
     * multiple values, the order of those values in the arrays (expectedValues, tolerances,
     * measuredValues) must all be the same.
     *
     * @param expectedValues A function that returns your expected/target values as an array.
     * @param tolerance A function that returns your tolerances as an array.
     * @param measuredValues A function that returns your measured values as an array.
     * @param cmd The command to run the check on.
     * @param time How many seconds after the start of the check it will sample at. The whole sequence
     *     will run for 2x this time.
     * @param name The name of the check.
     */
    public static Command goalCheck(
            double[] targets,
            double tolerance,
            double startingValue,
            Function<Double, Command> setGoal,
            Function<Double, Trigger> atGoal,
            double timeout,
            String name) {
        Supplier<Command>[] goToPoses = new Supplier[targets.length];

        Supplier<Command> goToStartingPose = () -> setGoal.apply(startingValue)
                .asProxy()
                .andThen(new WaitUntilCommand(atGoal.apply(startingValue)))
                .finallyDo(() -> {
                    if (atGoal.apply(startingValue).getAsBoolean()) {
                        Logger.recordOutput("Pitchecks/" + name + "/goToGoal/starting", true);
                    } else {
                        Logger.recordOutput("Pitchecks/" + name + "/goToGoal/starting", false);
                    }
                })
                .beforeStarting(() -> {
                    Logger.recordOutput("Pitchecks/" + name + "/goToGoal/starting", false);
                })
                .withTimeout(timeout * 2);

        for (int i = 0; i < targets.length; i++) {
            int index = i;
            double target = targets[i];
            goToPoses[i] = () -> setGoal.apply(target)
                    .asProxy()
                    .andThen(new WaitUntilCommand(atGoal.apply(target)))
                    .finallyDo(() -> {
                        Logger.recordOutput(
                                "Pitchecks/" + name + "/goToGoal/" + String.valueOf(index),
                                atGoal.apply(target).getAsBoolean());
                    })
                    .beforeStarting(() -> {
                        Logger.recordOutput("Pitchecks/" + name + "/goToGoal/" + String.valueOf(index), false);
                    })
                    .withTimeout(timeout * 2);
        }

        Command checkCommand = Commands.sequence();

        for (var goToPose : goToPoses) {
            checkCommand = checkCommand.andThen(goToPose.get());
        }

        checkCommand = checkCommand.andThen(goToStartingPose.get());

        for (var goToPose : goToPoses) {
            checkCommand = checkCommand.andThen(goToPose.get()).andThen(goToStartingPose.get());
        }

        return checkCommand;
    }
}
