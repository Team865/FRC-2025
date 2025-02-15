package ca.warp7.frc2025;

import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Class containing global constants for the whole robot
 *
 * CANID Schema for CANivoreCANivore bus:
 * 0_ system devices: gyro, power
 * 1_ Front Left  swerve
 * 2_ Front Right swerve
 * 3_ Back  Left  swerve
 * 4_ Back  Right swerve
 * 5_ Elevator
 * 6_ Climber
 *
 * CANID Schema for rio bus:
 * 0_ system deivces:
 * 1_ intake
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
        public static final int CLIMBER_LEFT_MOTER_ID = 0;
        public static final int CLIMBER_RIGHT_MOTER_ID = 0;
        public static final int CLIMBER_INTAKE_MOTER_ID = 0;
        public static final double CLIMBER_ARM_LENGTH_METERS = Units.inchesToMeters(0.1);
        public static final double CLIMBER_DRUM_RADIUS_METERS = Units.inchesToMeters(0.1);

        public static final double CLIMBER_PIVOT_GEAR_RATIO = 1.0;
        public static final double CLIMBER_INTAKE_GEAR_RATIO = 1.0;
    }

    public static final class Elevator {
        public static final double DRUM_RADIUS_METERS = 0.048514 / 2;
        public static final double GEAR_RATIO = 80 / 16;
    }
}
