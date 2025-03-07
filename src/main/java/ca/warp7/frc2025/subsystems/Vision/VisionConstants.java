// From
// https://github.com/Mechanical-Advantage/AdvantageKit/blob/cb7de552c144af211f63e22d76caddeb57cead40/template_projects/sources/vision/src/main/java/frc/robot/subsystems/vision/VisionConstants.java

package ca.warp7.frc2025.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.HashMap;

public class VisionConstants {
    public static Rotation2d[] tx = {Rotation2d.fromDegrees(0.002), Rotation2d.fromDegrees(0.001)};

    public static Rotation2d[] ty = {Rotation2d.fromRadians(-0.001), Rotation2d.fromRadians(-0.001)};

    public static HashMap<Integer, Tuple<Rotation2d, Rotation2d>> reefTy = new HashMap<>();

    public static record Tuple<L, R>(L l, R r) {}

    // first value is limelight on the right 2nd is the limelight on the left
    static {
        // Defaults

        // Red
        reefTy.put(6, new Tuple<>(Rotation2d.fromDegrees(-0.001), Rotation2d.fromDegrees(-0.001)));
        reefTy.put(7, new Tuple<>(Rotation2d.fromDegrees(-0.001), Rotation2d.fromDegrees(-0.001)));
        reefTy.put(8, new Tuple<>(Rotation2d.fromDegrees(-0.001), Rotation2d.fromDegrees(-0.001)));
        reefTy.put(9, new Tuple<>(Rotation2d.fromDegrees(-0.001), Rotation2d.fromDegrees(-0.001)));
        reefTy.put(10, new Tuple<>(Rotation2d.fromDegrees(-0.001), Rotation2d.fromDegrees(-0.001)));
        reefTy.put(11, new Tuple<>(Rotation2d.fromDegrees(-0.001), Rotation2d.fromDegrees(-0.001)));

        // Blue
        reefTy.put(17, new Tuple<>(Rotation2d.fromDegrees(-0.001), Rotation2d.fromDegrees(-0.001)));
        reefTy.put(18, new Tuple<>(Rotation2d.fromDegrees(-0.001), Rotation2d.fromDegrees(-0.001)));
        reefTy.put(19, new Tuple<>(Rotation2d.fromDegrees(-0.001), Rotation2d.fromDegrees(-0.001)));
        reefTy.put(20, new Tuple<>(Rotation2d.fromDegrees(-0.001), Rotation2d.fromDegrees(-0.001)));
        reefTy.put(21, new Tuple<>(Rotation2d.fromDegrees(-0.001), Rotation2d.fromDegrees(-0.001)));
        reefTy.put(22, new Tuple<>(Rotation2d.fromDegrees(-0.001), Rotation2d.fromDegrees(-0.001)));

        // At home field

        // reefTy.put(17, new Tuple<>(Rotation2d.fromDegrees(0.1), Rotation2d.fromDegrees(1.8)));
        // reefTy.put(18, new Tuple<>(Rotation2d.fromDegrees(-1.27), Rotation2d.fromDegrees(-0.80)));
        // reefTy.put(19, new Tuple<>(Rotation2d.fromDegrees(1), Rotation2d.fromDegrees(2.5)));
        // reefTy.put(20, new Tuple<>(Rotation2d.fromDegrees(0.45), Rotation2d.fromDegrees(2)));
        // reefTy.put(21, new Tuple<>(Rotation2d.fromDegrees(0.75), Rotation2d.fromDegrees(2.51)));
        // reefTy.put(22, new Tuple<>(Rotation2d.fromDegrees(1.58), Rotation2d.fromDegrees(2.69)));

        // Cent field
        reefTy.put(6, new Tuple<>(Rotation2d.fromDegrees(1.82), Rotation2d.fromDegrees(3.09)));
        reefTy.put(7, new Tuple<>(Rotation2d.fromDegrees(1.61), Rotation2d.fromDegrees(2.95)));
        reefTy.put(8, new Tuple<>(Rotation2d.fromDegrees(1.67), Rotation2d.fromDegrees(3.27)));
        reefTy.put(9, new Tuple<>(Rotation2d.fromDegrees(1.83), Rotation2d.fromDegrees(3.07)));
        reefTy.put(10, new Tuple<>(Rotation2d.fromDegrees(1.60), Rotation2d.fromDegrees(3.15)));
        reefTy.put(11, new Tuple<>(Rotation2d.fromDegrees(1.53), Rotation2d.fromDegrees(2.90)));

        reefTy.put(17, new Tuple<>(Rotation2d.fromDegrees(1.59), Rotation2d.fromDegrees(3.04)));
        reefTy.put(18, new Tuple<>(Rotation2d.fromDegrees(1.63), Rotation2d.fromDegrees(3.26)));
        reefTy.put(19, new Tuple<>(Rotation2d.fromDegrees(1.78), Rotation2d.fromDegrees(3.35)));
        reefTy.put(20, new Tuple<>(Rotation2d.fromDegrees(1.43), Rotation2d.fromDegrees(3.12)));
        reefTy.put(21, new Tuple<>(Rotation2d.fromDegrees(1.85), Rotation2d.fromDegrees(3.24)));
        reefTy.put(22, new Tuple<>(Rotation2d.fromDegrees(1.48), Rotation2d.fromDegrees(3.22)));
    }

    public static Rotation2d getTy(int id, int camera) {
        Tuple<Rotation2d, Rotation2d> tys = reefTy.get(id);

        if (camera == 0) {
            return tys.l();
        } else {
            return tys.r();
        }
    }

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static String camera0Name = "limelight-right";
    public static String camera1Name = "limelight-left";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 = new Transform3d(0.17858, -0.16464, 0.259, new Rotation3d(0.0, -13, 0.0));
    public static Transform3d robotToCamera1 = new Transform3d(
            0.17858, // -0.173683,
            0.16464, // 0.178485,
            0.259, // 0.260248,
            // new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(15)));
            new Rotation3d(0, Units.degreesToRadians(-13), 0));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
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
