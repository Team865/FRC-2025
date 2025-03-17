package ca.warp7.frc2025.subsystems.climber;

import ca.warp7.frc2025.Constants.Climber;
import ca.warp7.frc2025.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs;
    public double goal = 0.0;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
        inputs = new ClimberIOInputsAutoLogged();
        io.setPD(Climber.kPNormal, Climber.kDNormal);
    }

    @AutoLogOutput(key = "Climber/goal")
    public Command setGoal(double goal) {
        return this.runOnce(() -> io.setPivotPosition(goal));
    }

    public Command setPivotVoltage(double volts) {
        return this.runOnce(() -> io.setPivotVoltage(volts));
    }

    public Command setPivotServoPosition(double position) {
        return this.runOnce(() -> io.setServoPosition(position));
    }

    public Command setPivotSpeed(double speed) {
        return this.runOnce(() -> io.setPivotSpeed(speed));
    }

    public Command setPivotPosition(double position) {
        return this.runOnce(() -> io.setPivotPosition(position));
    }

    public Trigger atSetpointTrigger() {
        return new Trigger(() -> this.goal == inputs.pivotRotation);
    }

    public Command setNormalGains() {
        return runOnce(() -> io.setPD(Climber.kPNormal, Climber.kDNormal));
    }

    public Command setClimbGains() {
        return runOnce(() -> io.setPD(Climber.kPClimbing, Climber.kDClimbing));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        Logger.recordOutput("Climber/goal", goal);

        if (atSetpointTrigger().getAsBoolean()) {
            io.stop();
        }
    }
}
