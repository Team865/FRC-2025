package ca.warp7.frc2025.util;

import ca.warp7.frc2025.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

public class ClosestHPStation {
    private static List<Pose2d> stations =
            List.of(FieldConstants.CoralStation.leftCenterFace, FieldConstants.CoralStation.rightCenterFace);

    public static Pose2d getClosestStation(Pose2d pose) {
        return pose.nearest(stations);
    }
}
