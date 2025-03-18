package ca.warp7.frc2025.subsystems.Vision;

import static ca.warp7.frc2025.subsystems.Vision.VisionConstants.*;

import ca.warp7.frc2025.subsystems.Vision.VisionIO.PoseObservation;
import ca.warp7.frc2025.subsystems.Vision.VisionIO.PoseObservationType;
import ca.warp7.frc2025.subsystems.Vision.VisionIO.TargetObservation;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public final List<Integer> tags = new ArrayList<Integer>();

    public VisionSubsystem(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }
        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                    new Alert("Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera " + inputs[i].name, inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        if (inputs[0].tagIds.length > 0 || inputs[1].tagIds.length > 0) {
            tags.clear();
        }

        for (int index = 0; index < io.length; index++) {
            disconnectedAlerts[index].set(!inputs[index].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[index].tagIds) {
                tags.add(tagId);

                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[index].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth()
                        || observation.type() == PoseObservationType.MEGATAG_1;

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;

                linearStdDev *= linearStdDevMegatag2Factor;
                angularStdDev *= angularStdDevMegatag2Factor;

                if (index < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[index];
                    angularStdDev *= cameraStdDevFactors[index];
                }

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }
            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(index) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(index) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(index) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(index) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);

            // Log summary data
            Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
            Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Summary/RobotPosesAccepted",
                    allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Summary/RobotPosesRejected",
                    allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

            Logger.recordOutput(
                    "Vision/tags", tags.stream().mapToInt((o) -> (int) o).toArray());
        }
    }

    @AutoLogOutput
    public Optional<Integer> getTagID(int camera) {
        if (inputs[camera].tagIds.length > 0) {
            return Optional.of(inputs[camera].tagIds[0]);
        } else {
            return Optional.empty();
        }
    }

    @AutoLogOutput
    public int[] getTagIDS(int camera) {
        return tags.stream().mapToInt(i -> (int) i).toArray();
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public TargetObservation getTarget(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation;
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public PoseObservation getPoseObv(int cameraIndex) {
        if (inputs[cameraIndex].poseObservations.length > 0) {
            return inputs[cameraIndex].poseObservations[0];
        } else {
            return null;
        }
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
