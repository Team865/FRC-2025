package ca.warp7.frc2025.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
    @AutoLog
    static class ClimberIOInputs {
        public Rotation2d pivotRotation = new Rotation2d(); 
        public double pivotVelocityRotationsPerSecond = 0.0; 

        public boolean motorConnected = false;
        public boolean followorConnected = false; 

        public double[] pivtoVoltage = new double[] {}; 
        public double[] pivotCurrentAmps = new double [] {}; 
        public double[] pivotTempCelsuis = new double[] {}; 

        
    }

    default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    default void setPivotSetpoint(final Rotation2d rotation){}

    default void setPivotVoltage(final double leftVoltage, final double rightVoltage) {}


}
