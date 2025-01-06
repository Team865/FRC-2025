package ca.warp7.frc2025.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    private final String name;

    public Module(ModuleIO io, String name, SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.io = io;
        this.name = name;
        this.constants = constants;

        driveDisconnectedAlert = new Alert("Drive motor disconnected on module " + name, AlertType.kError);
        turnDisconnectedAlert = new Alert("Turn motor disconnected on module " + name, AlertType.kError);
        turnEncoderDisconnectedAlert = new Alert("Absolute encoder disconnected on module" + name, AlertType.kError);
    }

    public void runSetpoint(SwerveModuleState state) {
        // Optimize the new setpoint
        state.optimize(getAngle());
        state.cosineScale(inputs.turnPosition);

        // Apply setpoints
        io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
        io.setTurnPosition(state.angle);
    }

    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + name, inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
            Rotation2d angle = inputs.odometryTurnsPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    }
}
