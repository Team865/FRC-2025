package ca.warp7.frc2025.util;

import static edu.wpi.first.units.Units.Meter;

import ca.warp7.frc2025.Constants.Drivetrain;
import ca.warp7.frc2025.FieldConstants;
import ca.warp7.frc2025.subsystems.Vision.VisionConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class FieldConstantsHelper {
    private static List<Pose2d> stations = List.of(
            FieldConstants.CoralStation.leftCenterFace,
            FieldConstants.CoralStation.rightCenterFace,
            FieldConstants.CoralStation.rightCenterFaceRed,
            FieldConstants.CoralStation.leftCenterFaceRed);

    private static List<Pose2d> faces = List.of(FieldConstants.Reef.centerFaces);
    // private static List<Pose2d> facesRed = List.of(FieldConstants.Reef.centerFacesBlue);

    public static Pose2d getClosestStation(Pose2d pose) {
        Pose2d station = pose.nearest(stations);
        Logger.recordOutput("AutoAlign/hpStation", station);
        Logger.recordOutput("AutoAlign/hpStationFlip", AllianceFlipUtil.apply(station));
        return station;
    }

    public static Pose2d getclosestFace(Pose2d pose) {
        return pose.nearest(faces);
    }

    public static Pose2d faceToRobotPose(Pose2d pose, boolean left) {
        Distance yOffset =
                Meter.of(left ? VisionConstants.robotToCamera0.getY() : VisionConstants.robotToCamera1.getY());
        Transform2d transformer =
                new Transform2d(new Translation2d(Drivetrain.LENGTH.div(2), yOffset), Rotation2d.k180deg);

        pose = pose.transformBy(transformer);

        return pose;
    }
}
