package ca.warp7.frc2025.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnsPositions = new Rotation2d[] {};
    }

    /**
     * Updates the loggable inputs
     * @param inputs
     */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /**
     * Run the drive motor at the passed open loop value
     * @param output
     */
    public default void setDriveOpenLoop(double output) {}

    public default void setDrivePD(double P, double D) {}

    public default void setTurnPD(double P, double D) {}

    /**
     * Run the turn motor at the passed open loop value
     * @param output
     */
    public default void setTurnOpenLoop(double output) {}

    /**
     * Run the drive motor at the specified velocity.
     * @param velocityRadPerSec
     */
    public default void setDriveVelocity(double velocityRadPerSec) {}

    /**
     * Run the turn motor to the specified rotation.
     * @param rotation
     */
    public default void setTurnPosition(Rotation2d rotation) {}
}
