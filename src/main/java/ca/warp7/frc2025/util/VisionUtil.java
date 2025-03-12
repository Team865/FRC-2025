package ca.warp7.frc2025.util;

import static edu.wpi.first.units.Units.Meter;

import ca.warp7.frc2025.Constants.Drivetrain;
import ca.warp7.frc2025.subsystems.Vision.VisionConstants;
import ca.warp7.frc2025.subsystems.Vision.VisionSubsystem;
import ca.warp7.frc2025.subsystems.drive.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionUtil {
    /* Checks if a tag is is a valid reading, based on the robot's rotation */
    /* assumes id is a valid tag id */
    public static boolean validId(int id, Rotation2d pose) {
        return MathUtil.isNear(
                VisionConstants.aprilTagLayout
                        .getTagPose(id)
                        .get()
                        .getRotation()
                        .toRotation2d()
                        .getDegrees(),
                pose.getDegrees(),
                30);
    }

    public static Pose2d tagIdToRobotPose(int id, boolean left) {
        Pose2d pose = VisionConstants.aprilTagLayout.getTagPose(id).get().toPose2d();

        Distance yOffset =
                Meter.of(left ? VisionConstants.robotToCamera0.getY() : VisionConstants.robotToCamera1.getY());
        Transform2d transformer =
                new Transform2d(new Translation2d(Drivetrain.WIDTH.div(2), yOffset), Rotation2d.k180deg);

        pose = pose.transformBy(transformer);

        return pose;
    }

    public static boolean isReefTag(int id) {
        switch (id) {
            case 6 | 7 | 8 | 9 | 10 | 11 | 17 | 18 | 19 | 20 | 21 | 22:
                return true;
            default:
                return false;
        }
    }

    public static Optional<Integer> firstReefId(int[] ids) {
        for (int id : ids) {
            if (isReefTag(id)) {
                System.out.println(id);
                Logger.recordOutput("id", id);
                return Optional.of(id);
            }
        }

        return Optional.empty();
    }
}
