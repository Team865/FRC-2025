package ca.warp7.frc2025.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean motorConnected = false;

        public double pivotRotation = 0.0;
        public double pivotVelocityRotationsPerSecond = 0.0;
        public double pivotVoltage = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double pivotTempC = 0.0;
    }

    public default void updateInputs(final ClimberIOInputsAutoLogged inputs) {}

    public default void setPivotVoltage(final double volts) {}

    public default void setServoPosition(final double position) {}

    public default void setPivotSpeed(final double speed) {}

    public default void setPivotPosition(final double position) {}

    public default void setControlConstants(double kG, double kS, double kV, double kA, double kP, double kD) {}
}
