package ca.warp7.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import ca.warp7.frc2025.util.LoggedTunableNumber;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    // Logging
    private final ElevatorIO io;
    private final ElevatorIOInputAutoLogged inputs = new ElevatorIOInputAutoLogged();

    // Network
    private Mechanism2d mech;

    private final MechanismRoot2d root;

    // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // off the root node or another ligament object
    private final MechanismLigament2d elevator;

    private final MechanismLigament2d wrist;

    private final LoggedTunableNumber P = new LoggedTunableNumber("Elevator/P", 62.5);
    private final LoggedTunableNumber I = new LoggedTunableNumber("Elevator/I", 0);
    private final LoggedTunableNumber D = new LoggedTunableNumber("Elevator/D", 0);

    // Control
    private final PIDController feedbackController =
            new PIDController(P.getAsDouble(), I.getAsDouble(), D.getAsDouble());

    private final ElevatorFeedforward feedforwardController =
            new ElevatorFeedforward(0, 0.06, (DCMotor.getKrakenX60(2).KvRadPerSecPerVolt * 0.048514 / 2) / (80 / 14));

    private Distance goal = Inches.of(0);

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;

        mech = new Mechanism2d(3, 3);

        root = mech.getRoot("climber", 2, 0);

        elevator = root.append(new MechanismLigament2d("elevator", 0, 90));

        wrist = elevator.append(new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));

        SmartDashboard.putData("MyMechanism", mech);
    }

    public Command setGoal(Distance goal) {
        return this.runOnce(() -> this.goal = goal);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator/inputs", inputs);

        LoggedTunableNumber.ifChanged(hashCode(), (pid) -> feedbackController.setPID(pid[0], pid[1], pid[2]), P, I, D);
        double feedforwardVolts = feedforwardController.calculate(inputs.velocityMeters);

        double feedbackVolts = feedbackController.calculate(inputs.extensionMeters, goal.in(Meters));
        io.setVoltage(feedbackVolts + feedforwardVolts);

        elevator.setLength(inputs.extensionMeters);
    }
}
