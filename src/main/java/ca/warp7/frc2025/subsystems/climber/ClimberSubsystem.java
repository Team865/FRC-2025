package ca.warp7.frc2025.subsystems.climber;

import ca.warp7.frc2025.util.LoggedTunableNumber;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public static final Rotation2d PIVOT_MIN_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(0);

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Climber/P", 0);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Climber/I", 0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Climber/D", 0);

    private final LoggedTunableNumber kG = new LoggedTunableNumber("Climber/kG", 0);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Climber/kS", 0);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Climber/kV", 0);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Climber/kA", 0);

    private Rotation2d goal = new Rotation2d();

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public Rotation2d getAngle() {
        return inputs.pivotRotation;
    }

    public Command runStateCmd(Supplier<Rotation2d> rotation) {
        return this.run(() -> {
            goal = rotation.get();
            io.setPivotSetpoint(rotation.get());
        });
    }

    public Command runStateCmd(Rotation2d rotation) {
        return runStateCmd(() -> rotation);
    }

    public Command setPivotVoltage(double volts) {
        return this.run(() -> {
            io.setPivotVoltage(volts);
        });
    }

    public Command setIntakeVoltage(double volts) {
        return this.run(() -> {
            io.setIntakeVoltage(volts);
            System.out.println("INTAKE VOLTAGE");
        });
    }
}
