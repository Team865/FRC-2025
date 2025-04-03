package ca.warp7.frc2025.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import ca.warp7.frc2025.Constants.Drivetrain;
import ca.warp7.frc2025.FieldConstants;
import ca.warp7.frc2025.FieldConstants.Reef;
import ca.warp7.frc2025.subsystems.Vision.VisionConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class FieldConstantsHelper {
    private static List<Pose2d> stations = List.of(
            FieldConstants.CoralStation.leftCenterFace,
            FieldConstants.CoralStation.rightCenterFace,
            FieldConstants.CoralStation.rightCenterFaceRed,
            FieldConstants.CoralStation.leftCenterFaceRed);

    private static List<Pose2d> faces = List.of(FieldConstants.Reef.centerFaces);
    private static Translation2d center = FieldConstants.Reef.center;
    // private static List<Pose2d> facesRed = List.of(FieldConstants.Reef.centerFacesBlue);

    public static Pose2d getClosestStation(Pose2d pose) {
        Pose2d station = pose.nearest(stations);
        Logger.recordOutput("Field Constants/Closest Station", station);
        return station;
    }

    public static Pose2d getclosestFace(Pose2d pose) {
        Pose2d face = pose.nearest(faces);
        Logger.recordOutput("Field Constants/Closest Reef Face", face);
        return face;
    }

    public static Rotation2d getAngleToReefCenter(Pose2d pose) {
        Rotation2d angle = new Rotation2d(center.getX() - pose.getX(), center.getY() - pose.getY());

        Logger.recordOutput("Field Constants/Pose from center of reef", new Pose2d(pose.getTranslation(), angle));

        return angle;
    }

    @AutoLogOutput
    public static Distance lengthFromCenterOfReef(Pose2d pose) {
        double target = pose.getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center));
        Logger.recordOutput("Field Constants/Length From Ceneter Pose", target);

        return Meters.of(target);
    }

    public static Pose2d stationToRobot(Pose2d pose) {
        Transform2d transformer =
                new Transform2d(new Translation2d(Drivetrain.LENGTH.div(2), Meters.zero()), Rotation2d.k180deg);

        Pose2d result = pose.transformBy(transformer);

        Logger.recordOutput("Field Constants/Robot Station Pose", result);

        return result;
    }

    public static Pose2d faceToRobotPose(Pose2d pose, boolean left) {
        Distance yOffset =
                Meter.of(left ? VisionConstants.robotToCamera0.getY() : VisionConstants.robotToCamera1.getY());
        Transform2d transformer =
                new Transform2d(new Translation2d(Drivetrain.LENGTH.div(2), yOffset), Rotation2d.k180deg);

        pose = pose.transformBy(transformer);

        Logger.recordOutput("Field Constants/Robot Reef Pose", pose);

        return pose;
    }

    public static Pose2d getAlgaeGoalPose(Pose2d pose) {
        pose = getclosestFace(pose);

        Transform2d transformer =
                new Transform2d(new Translation2d(Drivetrain.LENGTH.div(2), Inches.of(2)), Rotation2d.k180deg);

        pose = pose.transformBy(transformer);

        Logger.recordOutput("Field Constants/Robot Algae Pose", pose);

        return pose;
    }
}
