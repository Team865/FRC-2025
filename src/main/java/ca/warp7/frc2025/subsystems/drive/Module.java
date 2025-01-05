package ca.warp7.frc2025.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final SwerveModuleConstants constants;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;

    private final String name;

    public Module(ModuleIO io, String name, SwerveModuleConstants constants) {
        this.io = io;
        this.name = name;
        this.constants = constants;

        driveDisconnectedAlert = new Alert("Drive motor disconnected on module " + name, AlertType.kError);
        turnDisconnectedAlert = new Alert("Turn motor disconnected on module " + name, AlertType.kError);
        turnEncoderDisconnectedAlert = new Alert("Absolute encoder disconnected on module" + name, AlertType.kError);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + name, inputs);

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
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
}
