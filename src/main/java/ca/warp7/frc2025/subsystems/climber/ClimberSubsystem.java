package ca.warp7.frc2025.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs;
    public double goal = 0.0;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
        inputs = new ClimberIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.setPivotPosition(goal);
    }

    public Command setGoal(double goal) {
        return this.runOnce(() -> this.goal = goal);
    }

    public Command setIntakeVoltage(double volts) {
        return this.runOnce(() -> io.setIntakeVoltage(volts));
    }
}
