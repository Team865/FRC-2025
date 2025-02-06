package ca.warp7.frc2025.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
    @AutoLog
    static class ClimberIOInputs {
        public boolean motorConnected = false;
        public boolean followerConnected = true;

        public double pivotPosition = 0.0;
        public double pivotVelocityMetersPerSec = 0.0;

        public double[] pivotAppliedVolts = new double[] {};
        public double[] pivotCurrentAmps = new double[] {};

        public double climbIntakeVolts = 0.0;
        public double climbIntakeCurrentAmps = 0.0;
    }

    default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    default void setPivotVoltage(double volts) {}

    default void setIntakeVoltage(double volts) {}
}
