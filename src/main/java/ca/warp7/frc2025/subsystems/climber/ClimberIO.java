package ca.warp7.frc2025.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    static class ClimberIOInputs {
        public boolean motorConnected = false;
        public boolean followerConnected = false;

        public Rotation2d pivotRotation = new Rotation2d();
        public double pivotVelocityRotationsPerSecond = 0.0;

        public double[] pivotVoltage = new double[] {};
        public double[] pivotCurrentAmps = new double[] {};
        public double[] pivotTempCelsius = new double[] {};
    }

    public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    public default void setPivotSetpoint(final Rotation2d rotation) {}

    public default void setPivotVoltage(final double volts) {}
}
