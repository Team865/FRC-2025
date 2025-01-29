package ca.warp7.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInput {
        boolean encoderConnected = false;
        double velocityMeters = 0;
        double extensionMeters = 0;
        double volts = 0.0;
    }

    public abstract void updateInputs(ElevatorIOInputAutoLogged inputs);

    public abstract void setVoltage(double volts);
}
