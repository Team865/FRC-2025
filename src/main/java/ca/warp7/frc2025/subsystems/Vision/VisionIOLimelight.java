// Largely from
// https://github.com/Mechanical-Advantage/AdvantageKit/blob/cb7de552c144af211f63e22d76caddeb57cead40/template_projects/sources/vision/src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java

package ca.warp7.frc2025.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {
    private String limeLightName;
    private final Supplier<Rotation2d> rotationSupplier;

    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleArrayPublisher orientationPublisher;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;

    private final boolean flip;

    public VisionIOLimelight(String limeLightName, Supplier<Rotation2d> rotationSupplier, boolean flip) {
        this.limeLightName = limeLightName;
        NetworkTable nt = NetworkTableInstance.getDefault().getTable(limeLightName);
        this.rotationSupplier = rotationSupplier;
        orientationPublisher = nt.getDoubleArrayTopic("robot_orientation_set").publish();
        latencySubscriber = nt.getDoubleTopic("tl").subscribe(0.0);
        txSubscriber = nt.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = nt.getDoubleTopic("ty").subscribe(0.0);
        megatag1Subscriber = nt.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        megatag2Subscriber = nt.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

        this.flip = flip;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

        inputs.latestTargetObservation = new TargetObservation(
                Rotation2d.fromDegrees(txSubscriber.get() * (flip ? -1 : 1)),
                Rotation2d.fromDegrees(tySubscriber.get()));

        inputs.name = limeLightName;

        orientationPublisher.accept(new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});

        NetworkTableInstance.getDefault().flush(); // can hurt network traffic

        // Read new pose observations from NetworkTables
        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var rawSample : megatag1Subscriber.readQueue()) {
            if (rawSample.value.length == 0) continue;
            for (int i = 11; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            poseObservations.add(new PoseObservation(
                    // Timestamp, based on server timestamp of publish and latency
                    rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

                    // 3D pose estimate
                    parsePose(rawSample.value),

                    // Ambiguity, using only the first tag because ambiguity isn't applicable for multitag
                    rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

                    // Tag count
                    (int) rawSample.value[7],

                    // Average tag distance
                    rawSample.value[9],

                    // Observation type
                    PoseObservationType.MEGATAG_1));
        }
        for (var rawSample : megatag2Subscriber.readQueue()) {
            if (rawSample.value.length == 0) continue;
            for (int i = 11; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            poseObservations.add(new PoseObservation(
                    // Timestamp, based on server timestamp of publish and latency
                    rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

                    // 3D pose estimate
                    parsePose(rawSample.value),

                    // Ambiguity, zeroed because the pose is already disambiguated
                    0.0,

                    // Tag count
                    (int) rawSample.value[7],

                    // Average tag distance
                    rawSample.value[9],
                    PoseObservationType.MEGATAG_2));
        }
        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }

    /** Parses the 3D pose from a Limelight botpose array. */
    private static Pose3d parsePose(double[] rawLLArray) {
        return new Pose3d(
                rawLLArray[0],
                rawLLArray[1],
                rawLLArray[2],
                new Rotation3d(
                        Units.degreesToRadians(rawLLArray[3]),
                        Units.degreesToRadians(rawLLArray[4]),
                        Units.degreesToRadians(rawLLArray[5])));
    }
}
