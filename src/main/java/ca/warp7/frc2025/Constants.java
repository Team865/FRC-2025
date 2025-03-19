package ca.warp7.frc2025;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
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
    public static final boolean tuningMode = false;

    public static final double PERIOD = 0.02;

    public static boolean disableHAL = false;

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
        public static final Distance WIDTH = Meters.of(0.8763);
        public static final Distance LENGTH = Meters.of(0.978);
        public static final double ROBOT_MOI_SI = 6.8554906507;
        public static final double WHEEL_COF = 1.1;
    }

    public static final class Elevator {
        public static final int LEFT_MOTOR_ID = 0;
        public static final int RIGHT_MOTOR_ID = 0;

        public static final double DRUM_RADIUS_METERS = 0.048514 / 2;
        public static final double GEAR_RATIO = 80 / 16;

        public static final Distance STOW = Inches.of(0);
        public static final Distance L4 = Inches.of(28.5);
        public static final Distance INTAKE = Inches.of(3);
        public static final Distance L3 = Inches.of(19.625 - 0.75);
        public static final Distance L2 = Inches.of(11.75);
        // public static final Distance L2A = Inches.of(12);
        public static final Distance L2A = Inches.of(11.00);
        public static final Distance L1A = Inches.of(9.9);
    }

    public static final class Intake {
        public static final CANBus CANBUS = new CANBus("rio");

        public static final int MOTOR_ID = 11;

        public static record LaserCANConstants(int CANid, String name) {}

        public static final LaserCANConstants TOP_LASER_CAN = new LaserCANConstants(12, "Top");
        public static final LaserCANConstants FRONT_LASER_CAN = new LaserCANConstants(13, "Front");
    }

    public static final class Climber {
        public static final int PIVOT_ID = 61;
        public static final int INTAKE_ID = 58;
        public static final int Servo_PWM = 1;

        public static final Rotation2d DOWN = Rotation2d.fromDegrees(90);
        public static final Rotation2d CLIMB = Rotation2d.fromDegrees(0);
        public static final Rotation2d STOW = Rotation2d.kZero;

        public static final double kPNormal = 110.0;
        public static final double kDNormal = 2.5;

        public static final double kPClimbing = 200.0;
        public static final double kDClimbing = 2.5;
    }
}
