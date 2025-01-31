package ca.warp7.frc2025.subsystems.Climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    static class ClimberIOInputs {
        public Rotation2d pivotRotation = new Rotation2d();
        public double pivotVelocityRadPerSec = 0.0;

        public double[] pivotAppliedVolts = new double[] {};
        public double[] pivotCurrentAmps = new double[] {};

        public double climbIntakeVelocityRadPerSec = 0.0;
        public double climbIntakeVolts = 0.0;
        public double climbIntakeCurrentAmps = 0.0;
    }

    default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    default void setPivotVoltage(double volts) {}

    default void setPivotPosition(Rotation2d rotation) {}

    default void resetPivotPosition(Rotation2d rotation) {}

    default void setIntakeVoltage(double volts) {}
}
