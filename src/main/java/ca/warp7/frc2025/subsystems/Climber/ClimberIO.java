package ca.warp7.frc2025.subsystems.Climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    static class ClimberIOInputs {
        public double velocityRadPerSec = 0.0;
        public Rotation2d position = new Rotation2d();
        public double climberAppliedVolts = 0.0;
        public double climberCurrentAmps = 0.0;
    }

    default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    default void setVoltage(double volts) {}

    default void setPosition(double position) {}
}
