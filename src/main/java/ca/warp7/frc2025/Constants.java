package ca.warp7.frc2025;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Class containing global constants for the whole robot
 */
public final class Constants {
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static final double PERIOD = 0.02;

    public static enum Mode {
        // Running on a real robot
        REAL,

        //  Running in the simulator
        SIM,

        // Replaying log file on the robot
        REPLAY
    }

    public static final class Drivetrain {
        public static final double PERIOD = 0.02;
    }
}
