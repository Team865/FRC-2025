package ca.warp7.frc2025.subsystems.elevator;

import ca.warp7.frc2025.Constants.Elevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim sim = new ElevatorSim(
            4.62, 0.07, DCMotor.getKrakenX60Foc(2), Units.inchesToMeters(0), Units.inchesToMeters(29), true, 0);

    private double appliedVolts = 0.0;

    private TrapezoidProfile profile = new TrapezoidProfile(new Constraints(0, 0));

    private final PIDController feedbackController = new PIDController(0, 0, 0);

    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward(
            0, 0.06, (DCMotor.getKrakenX60(1).KvRadPerSecPerVolt * Elevator.DRUM_RADIUS_METERS) / Elevator.GEAR_RATIO);

    private State goal = new TrapezoidProfile.State(0, 0);

    @Override
    public void updateInputs(ElevatorIOInputAutoLogged inputs) {
        State next = profile.calculate(0.02, new State(inputs.positionMeters, inputs.velocityMetersPerSec), goal);

        sim.setInputVoltage(feedbackController.calculate(inputs.positionMeters, goal.position)
                + feedforwardController.calculate(next.velocity));
        sim.update(0.02);

        inputs.positionMeters = sim.getPositionMeters();
        inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    }

    @Override
    public void setControlConstants(double kG, double kS, double kV, double kA, double kP, double kD) {
        feedbackController.setPID(kP, 0, kD);

        feedforwardController = new ElevatorFeedforward(kG, kS, kV, kA);
    }

    @Override
    public void setMotionProfile(double velocity, double acceleration, double jerk) {
        profile = new TrapezoidProfile(new Constraints(velocity, acceleration));
    }

    @Override
    public void setPosition(double meters) {
        goal = new State(meters, 0);
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12, 12);
        sim.setInput(appliedVolts);
    }
}
