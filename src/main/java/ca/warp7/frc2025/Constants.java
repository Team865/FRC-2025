package ca.warp7.frc2025;

import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Class containing global constants for the whole robot
 */
public final class Constants {
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
    public static final boolean tuningMode = true;

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

        public static final Mass ROBOT_MASS = Pounds.of(137.965);
        public static final double ROBOT_MOI_SI = 6;
        public static final double WHEEL_COF = 1.1;
    }

    public static final class Climber {
        public static final double CLIMBER_ARM_LENGTH_METERS = Units.inchesToMeters(0.0);
        public static final double CLIMBER_DRUM_RADIUS_METERS = Units.inchesToMeters(0.0);
        public static final double CLIMBER_GEAR_RATIO = 0.0;
    }

    public static final class Elevator {
        public static final double DRUM_RADIUS_METERS = 0.048514 / 2;
        public static final double GEAR_RATIO = 80 / 14;
    }
}
