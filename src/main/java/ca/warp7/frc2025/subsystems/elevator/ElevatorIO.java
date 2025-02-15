package ca.warp7.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInput {
        boolean motorConnected = false;
        boolean followerConnected = false;
        double positionMeters = 0.0;
        double velocityMetersPerSec = 0.0;
        double[] appliedVolts = new double[] {};
        double[] torqueCurrentAmps = new double[] {};
        double[] supplyCurrentAmps = new double[] {};
        double[] tempCelsius = new double[] {};
    }

    public default void updateInputs(ElevatorIOInputAutoLogged inputs) {}

    public default void setVoltage(double volts) {}

    public default void setPosition(double meters) {}

    public default void setControlConstants(double kG, double kS, double kV, double kA, double kP, double kD) {}

    public default void setMotionProfile(double velocity, double acceleration, double jerk) {}
}
