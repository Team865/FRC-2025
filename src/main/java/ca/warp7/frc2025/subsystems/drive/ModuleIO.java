package ca.warp7.frc2025.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public abstract class ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double odometryTimestamps = 0.0;
        public double odometryDrivePositionsRad = 0.0;
        public Rotation2d odometryTurnsPositions = new Rotation2d();
    }

    /**
     * Updates the loggable inputs
     * @param inputs
     */
    public abstract void updateInputs(ModuleIOInputs inputs);

    /**
     * Run the drive motor at the passed open loop value
     * @param output
     */
    public abstract void setDriveOpenLoop(double output);

    /**
     * Run the turn motor at the passed open loop value
     * @param output
     */
    public abstract void setTurnOpenLoop(double output);

    /**
     * Run the drive motor at the specified velocity.
     * @param velocityRadPerSec
     */
    public abstract void setDriveVelocity(double velocityRadPerSec);

    /**
     * Run the turn motor to the specified rotation.
     * @param rotation
     */
    public abstract void setTurnPosition(Rotation2d rotation);
}