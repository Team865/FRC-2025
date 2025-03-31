// From
// https://github.com/Mechanical-Advantage/AdvantageKit/blob/cb7de552c144af211f63e22d76caddeb57cead40/template_projects/sources/vision/src/main/java/frc/robot/subsystems/vision/VisionConstants.java

package ca.warp7.frc2025.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static String camera0Name = "limelight-right";
    public static String camera1Name = "limelight-left";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
            new Transform3d(0.17858, -0.16464, 0.259, new Rotation3d(0.0, Units.degreesToRadians(13), 0.0));
    public static Transform3d robotToCamera1 = new Transform3d(
            0.17858, // -0.173683,
            0.16464, // 0.178485,
            0.259, // 0.260248,
            // new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(15)));
            new Rotation3d(0, Units.degreesToRadians(13), 0));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.01; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
