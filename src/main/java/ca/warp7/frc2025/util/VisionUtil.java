package ca.warp7.frc2025.util;

import static edu.wpi.first.units.Units.Meter;

import ca.warp7.frc2025.Constants.Drivetrain;
import ca.warp7.frc2025.subsystems.Vision.VisionConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.Optional;

public class VisionUtil {
    /* Checks if a tag is is a valid reading, based on the robot's rotation */
    /* assumes id is a valid tag id */
    public static boolean validTag(int id, Rotation2d pose) {
        return MathUtil.isNear(
                VisionConstants.aprilTagLayout
                        .getTagPose(id)
                        .get()
                        .getRotation()
                        .toRotation2d()
                        .getDegrees(),
                pose.rotateBy(Rotation2d.k180deg).getDegrees(),
                30,
                -180,
                180);
    }

    public static Pose2d tagIdToRobotPose(int id, boolean left) {
        Pose2d pose = VisionConstants.aprilTagLayout.getTagPose(id).get().toPose2d();

        Distance yOffset =
                Meter.of(left ? VisionConstants.robotToCamera0.getY() : VisionConstants.robotToCamera1.getY());
        Transform2d transformer =
                new Transform2d(new Translation2d(Drivetrain.LENGTH.div(2), yOffset), Rotation2d.k180deg);

        pose = pose.transformBy(transformer);

        return pose;
    }

    public static boolean isReefTag(int id) {
        return (6 <= id && id <= 11) || (17 <= id && id <= 22);
    }

    public static Optional<Integer> firstValidReefId(int[] ids, Rotation2d rot) {
        for (int id : ids) {
            if (isReefTag(id) && validTag(id, rot)) {
                // System.out.println(id);
                // Logger.recordOutput("id", id);
                return Optional.of(id);
            }
        }

        return Optional.empty();
    }
}
