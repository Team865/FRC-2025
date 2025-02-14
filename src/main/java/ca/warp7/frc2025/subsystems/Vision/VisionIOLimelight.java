package ca.warp7.frc2025.subsystems.Vision;

import ca.warp7.frc2025.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
    private String limeLightName;

    public VisionIOLimelight(String limeLightName) {
        this.limeLightName = limeLightName;
    }

    @Override
    public void updateInputs(visionIOInputs inputs) {}

    @Override
    public void setPipelline(int index) {
        LimelightHelpers.setPipelineIndex(limeLightName, index);
    }
}
