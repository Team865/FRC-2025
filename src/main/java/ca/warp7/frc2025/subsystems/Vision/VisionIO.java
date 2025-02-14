package ca.warp7.frc2025.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class visionIOInputs {}

    default void updateInputs(visionIOInputs inputs) {}

    default void setPipelline(int index) {}
}
