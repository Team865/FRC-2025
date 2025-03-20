package ca.warp7.frc2025.util.pitchecks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public interface PitCheckable {
    Command check();

    /**
     * Runs a pit check that ramps to some target value (position, speed, etc.).
     *
     * @param rampRate the amount to ramp by in units per second.
     * @param max the value to ramp towards.
     * @param tolerance the tolerance of the target value.
     * @param actualValue A function that returns the value that is being checked.
     * @param setValue A function sets the value that is being checked.
     * @param timeout How many seconds after the start of the check it will sample at. The whole sequence
     *     will run for 2x this time.
     * @param name The name of the check.
     */
    default Command rampCheck(
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
                Logger.recordOutput("Pitchecks/" + name + "/rampForward", "Success");
            } else {
                Logger.recordOutput("Pitchecks/" + name + "/rampForward", "Failure");
            }
        });
        Command reverseCheck = Commands.runOnce(() -> {
            if (MathUtil.isNear(max, actualValue.get(), tolerance)) {
                Logger.recordOutput("Pitchecks/" + name + "/rampReverse", "Success");
            } else {
                Logger.recordOutput("Pitchecks/" + name + "/rampReverse", "Failure");
            }
        });

        return Commands.runOnce(() -> {
                    limter.reset(startingValue);
                    Logger.recordOutput("Pitchecks/" + name + "/rampForward", "Unknown");
                    Logger.recordOutput("Pitchecks/" + name + "/rampReverse", "Unknown");
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
    // default Command check(
    //         String name,
    //         double rampRate,
    //         double min,
    //         double max,
    //         DoubleSupplier checkedValueSupplier,
    //         double tolerance,
    //         double timeout,
    //         SubsystemBase subsystem) {
    //     SlewRateLimiter limter = new SlewRateLimiter(rampRate);
    //
    //     return Commands.runOnce(() -> {
    //                 limter.reset(min);
    //             })
    //             .andThen(Commands.run(
    //                             () -> {
    //                                 double position = limter.calculate(max);
    //                                 setCheckedValue(position);
    //                             },
    //                             subsystem)
    //                     .until(() -> MathUtil.isNear(max, checkedValueSupplier.getAsDouble(), tolerance))
    //                     .andThen(Commands.runOnce(() -> setCheckedValue(min), subsystem)));
    // }
}
