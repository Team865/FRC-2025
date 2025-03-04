package ca.warp7.frc2025.subsystems.climber;

import ca.warp7.frc2025.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs;
    public double goal = 0.0;

    private final LoggedTunableNumber kG = new LoggedTunableNumber("Climber/kG", 0.0);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Climber/kS", 0.0);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Clibmer/kV", 0.0);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Climber/kA", 0.0);

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Climber/kP", 50);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Climber/kD", 10);

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
        inputs = new ClimberIOInputsAutoLogged();
        io.setControlConstants(kG.get(), kS.get(), kV.get(), kA.get(), kP.get(), kD.get());
    }

    @AutoLogOutput(key = "Climber/goal")
    public Command setGoal(double goal) {
        return this.runOnce(() -> io.setPivotPosition(goal));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        Logger.recordOutput("Climber/goal", goal);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                (constants) -> io.setControlConstants(
                        constants[0], constants[1], constants[2], constants[3], constants[4], constants[5]),
                kG,
                kS,
                kV,
                kA,
                kP,
                kD);
    }

    public Command setIntakeVoltage(double volts) {
        return this.runOnce(() -> io.setIntakeVoltage(volts));
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
}
