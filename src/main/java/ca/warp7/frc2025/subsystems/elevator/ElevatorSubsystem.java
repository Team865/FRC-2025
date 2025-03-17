package ca.warp7.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import ca.warp7.frc2025.Constants.Elevator;
import ca.warp7.frc2025.util.LoggedTunableNumber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

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

    @AutoLogOutput
    private Distance goal = Inches.of(0);

    private final SysIdRoutine sysIdRoutine;

    // Visualization
    @AutoLogOutput
    private final LoggedMechanism2d canvas = new LoggedMechanism2d(3, 3);

    private final LoggedMechanismRoot2d root = canvas.getRoot("root", 1.7, 0);
    private final LoggedMechanismLigament2d elevatorLigament = root.append(new LoggedMechanismLigament2d(
            "elevator", Units.inchesToMeters(7.531), 81, 11, new Color8Bit(Color.kOrange)));
    private final LoggedMechanismLigament2d intakeLigament = elevatorLigament.append(
            new LoggedMechanismLigament2d("intake", Units.inchesToMeters(24.464), 0, 20, new Color8Bit(Color.kBlue)));

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        Volts.of(1),
                        Seconds.of(3),
                        (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.magnitude()), null, this));

        io.setControlConstants(kG.get(), kS.get(), kV.get(), kA.get(), kP.get(), kD.get());

        io.setMotionProfile(velocity.get(), accel.get(), accel.get());
    }

    public Command setGoal(Distance goal) {
        return this.runOnce(() -> this.goal = goal);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public BooleanSupplier atSetpoint() {
        return () -> MathUtil.isNear(goal.in(Meters), inputs.positionMeters, Units.inchesToMeters(0.25));
    }

    public BooleanSupplier atSetpoint(Distance goal) {
        return () -> MathUtil.isNear(goal.in(Meters), inputs.positionMeters, Units.inchesToMeters(0.25));
    }

    public Trigger atSetpointTrigger() {
        return new Trigger(atSetpoint());
    }

    public Trigger atSetpointTrigger(Distance goal) {
        return new Trigger(atSetpoint(goal));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator/inputs", inputs);

        elevatorLigament.setLength(Units.inchesToMeters(7.531) + inputs.positionMeters * 2);

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

        if (atSetpointTrigger(Elevator.STOW).getAsBoolean() && goal.isEquivalent(Elevator.STOW)) {
            io.stop();
        } else {
            io.setPosition(goal.in(Meters));
        }
    }
}
