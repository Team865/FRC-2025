package ca.warp7.frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
    @AutoLog
    public static class RollersIOInputs {
        public boolean connected = false;
        public double positionRads = 0.0;
        public double velocityRadsPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
    }

    default void updateInputs(RollersIOInputsAutoLogged inputs) {}

    default void setVolts(double volts) {}

    default void setTorqueAmps(double amps) {}
}
