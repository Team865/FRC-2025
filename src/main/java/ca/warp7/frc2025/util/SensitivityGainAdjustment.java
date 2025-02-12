package ca.warp7.frc2025.util;

public class SensitivityGainAdjustment {

    /**
     * Passes input through cubic function to effectively change  sensitivity of joystick
     *
     * @param input Raw drive gain
     * @return Adjusted drive gain
     */
    public static double driveGainAdjustment(double input) {
        return ((0.45) * (input)) + ((.55) * (Math.pow(input, 3)));
    }

    /**
     *  Passes input through cubic function tp effectively change sensitivity of joystick
     *
     * @param input Raw steer gain
     * @return Adjusted steer gain
     */
    public static double steerGainAdjustment(double input) {
        return ((0.75) * (input)) + ((.25) * (Math.pow(input, 3)));
    }
}
