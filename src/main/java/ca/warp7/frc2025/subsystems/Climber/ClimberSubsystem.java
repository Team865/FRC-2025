package ca.warp7.frc2025.subsystems.Climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public Command runVoltageCommand(double volts) {
        return this.runOnce(() -> io.setIntakeVoltage(volts));
    }

    public Command setPivotVoltage(double volts) {
        return this.runOnce(() -> io.setPivotVoltage(volts));
    }

}
