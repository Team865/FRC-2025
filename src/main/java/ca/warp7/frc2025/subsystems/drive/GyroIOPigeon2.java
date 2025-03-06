package ca.warp7.frc2025.subsystems.drive;

import ca.warp7.frc2025.util.PhoenixUtil;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

public class GyroIOPigeon2 implements GyroIO {
    // Devices
    private final Pigeon2 pigeon;

    // Status Signals
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;

    // Odometry Queues
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOPigeon2(int Pigeon2Id, String CANBus) {
        pigeon = new Pigeon2(Pigeon2Id, CANBus);

        Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPose = new MountPoseConfigs().withMountPoseYaw(180);

        PhoenixUtil.tryUntilOk(5, () -> pigeon.getConfigurator().apply(new Pigeon2Configuration(), 0.25));
        PhoenixUtil.tryUntilOk(5, () -> pigeon.getConfigurator().setYaw(0.0));

        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();

        yaw.setUpdateFrequency(DriveSubsystem.ODOMETRY_FREQUENCY);
        yawVelocity.setUpdateFrequency(50.0);

        pigeon.optimizeBusUtilization();

        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
