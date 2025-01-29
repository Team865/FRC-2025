package ca.warp7.frc2025.subsystems.Climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    static class ClimberIOInputs {
        public Rotation2d position = new Rotation2d();
        
        public double pivotLeftVelocityRadPerSec = 0.0;
        public double pivotLeftVolts = 0.0;
        public double pivotLeftCurrentAmps = 0.0;

        public double pivotRightVelocityRadPerSec = 0.0;
        public double pivotRightVolts = 0.0;
        public double pivotRightCurrentAmps = 0.0;

        public double climbIntakeVelocityRadPerSec = 0.0;
        public double climbIntakeVolts = 0.0;
        public double climbIntakeCurrentAmps = 0.0;
    }

    default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    default void setPivotVoltage(double left, double right) {}

    default void setPivotSetpoint(Rotation2d rotation) {}

    default void setPivotPosition(Rotation2d roatation) {}
    
    default void setClimbIntakeVoltage(double volts) {}
    
}
