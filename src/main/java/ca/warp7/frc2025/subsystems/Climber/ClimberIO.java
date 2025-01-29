package ca.warp7.frc2025.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
    @AutoLog 
    static class ClimberIOInputs {
        public double velocityRadPerSec = 0.0;
        public Rotation2d position = new Rotation2d();
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
 
    }

    default void updateInputs(ClimberIOInputs inputs){}

    default void setVoltage(double volts){}
    default void setPosition(double position){}

}
