package ca.warp7.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import ca.warp7.frc2025.util.LoggedTunableNumber;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    // Logging
    private final ElevatorIO io;
    private final ElevatorIOInputAutoLogged inputs = new ElevatorIOInputAutoLogged();

    private final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.56);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.24);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 4.44);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.07);

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 100);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);

    private final LoggedTunableNumber velocity = new LoggedTunableNumber("Elevator/MaxVel", 80);
    private final LoggedTunableNumber accel = new LoggedTunableNumber("Elevator/maxAccel", 200);
    private final LoggedTunableNumber jerk = new LoggedTunableNumber("Elevator/maxJerk", 0);
    // Control

    private Distance goal = Inches.of(0);

    private final SysIdRoutine sysIdRoutine;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, Volts.of(4), null, (state) -> Logger.recordOutput("Elevator/SysIdState/", state)),
                new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.magnitude()), null, this));

        io.setControlConstants(kG.get(), kS.get(), kV.get(), kA.get(), kP.get(), kD.get());

        io.setMotionProfile(velocity.get(), accel.get(), accel.get());
    }

    @AutoLogOutput(key = "Elevator/goal")
    public Command setGoal(Distance goal) {
        return this.runOnce(() -> this.goal = goal);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator/inputs", inputs);
        Logger.recordOutput("Elevator/goal", goal);

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
        LoggedTunableNumber.ifChanged(
                hashCode(),
                (constraints) -> io.setMotionProfile(
                        Units.inchesToMeters(constraints[0]),
                        Units.inchesToMeters(constraints[1]),
                        Units.inchesToMeters(constraints[2])),
                velocity,
                accel,
                jerk);

        io.setPosition(goal.in(Meters));
    }
}
