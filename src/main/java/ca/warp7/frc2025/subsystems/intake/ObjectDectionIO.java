package ca.warp7.frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDectionIO {
    @AutoLog
    public static class ObjectDectionIOInputs {
        double objectDistanceMM = 0.0;
    }

    default void updateInputs(ObjectDectionIOInputs inputs) {}
}
