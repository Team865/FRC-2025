package ca.warp7.frc2025.subsystems.superstructure;

import ca.warp7.frc2025.Constants.Elevator;
import ca.warp7.frc2025.FieldConstants.ReefLevel;
import ca.warp7.frc2025.subsystems.climber.ClimberSubsystem;
import ca.warp7.frc2025.subsystems.elevator.ElevatorSubsystem;
import ca.warp7.frc2025.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    public static enum SuperState {
        IDLE,
        INTAKE_CORAL,
        READY_CORAL,
        SPIT_CORAL,
        PRE_L1,
        PRE_L2,
        PRE_L3,
        PRE_L4,
        SCORE_CORAL,
        PRE_ALGAE_HIGH,
        PRE_ALGAE_LOW,
        CLEAR_ALGAE,
        PRE_CLIMB,
        CLIMB
    }

    @AutoLogOutput(key = "Superstructure/Intake Req")
    private final Trigger intakeReq;

    @AutoLogOutput(key = "Superstructure/Pre Score Req")
    private final Trigger preScoreReq;

    @AutoLogOutput(key = "Superstructure/Score Req")
    private final Trigger scoreReq;

    @AutoLogOutput(key = "Superstructure/Stow Req")
    private final Trigger stowReq;

    @AutoLogOutput(key = "Superstructure/Pre Climber Req")
    private final Trigger preClimberReq;

    @AutoLogOutput(key = "Superstructure/Climber Req")
    private final Trigger climberReq;

    @AutoLogOutput(key = "Superstructure/Elev Safe To Move")
    private final Trigger elevatorSafeToMove;

    @AutoLogOutput(key = "Superstructure/Reef Level")
    private ReefLevel reefLevel = ReefLevel.L4;

    private SuperState lastState = SuperState.IDLE;
    private SuperState state = SuperState.IDLE;
    private Map<SuperState, Trigger> stateTriggers = new HashMap<SuperState, Trigger>();

    private final ElevatorSubsystem elevator;
    private final IntakeSubsystem intake;
    private final ClimberSubsystem climber;

    public Superstructure(
            ElevatorSubsystem elevator,
            IntakeSubsystem intake,
            ClimberSubsystem climber,
            Trigger intakeReq,
            Trigger preScoreReq,
            Trigger scoreReq,
            Trigger stowReq,
            Trigger preClimberReq,
            Trigger climberReq,
            Trigger elevatorSafeToMove) {
        this.elevator = elevator;
        this.intake = intake;
        this.climber = climber;

        this.intakeReq = intakeReq;
        this.preScoreReq = preScoreReq;
        this.scoreReq = scoreReq;
        this.stowReq = stowReq;
        this.preClimberReq = preClimberReq;
        this.climberReq = climberReq;
        this.elevatorSafeToMove = elevatorSafeToMove;

        for (var state : SuperState.values()) {
            stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
        }

        configureStateTransitionCommands();
    }

    private void configureStateTransitionCommands() {
        stateTriggers.get(SuperState.IDLE).onTrue(elevator.setGoal(Elevator.STOW));

        stateTriggers
                .get(SuperState.IDLE)
                .or(stateTriggers.get(SuperState.READY_CORAL))
                .and(preClimberReq)
                .whileTrue(climber.down())
                .and(climber.atSetpointTrigger())
                .onTrue(forceState(SuperState.CLIMB));

        stateTriggers.get(SuperState.CLIMB).and(climberReq).whileTrue(climber.climb());

        stateTriggers.get(SuperState.IDLE).and(intakeReq).onTrue(forceState(SuperState.INTAKE_CORAL));

        stateTriggers
                .get(SuperState.INTAKE_CORAL)
                .whileTrue(intake.intake())
                .and(intake.holdingCoral())
                .onTrue(forceState(SuperState.READY_CORAL));

        stateTriggers
                .get(SuperState.READY_CORAL)
                .whileTrue(elevator.setGoal(Elevator.STOW))
                .and(preScoreReq)
                .and(() -> reefLevel == ReefLevel.L4)
                .onTrue(forceState(SuperState.PRE_L4));

        stateTriggers
                .get(SuperState.READY_CORAL)
                .whileTrue(elevator.setGoal(Elevator.STOW))
                .and(preScoreReq)
                .and(() -> reefLevel == ReefLevel.L3)
                .onTrue(forceState(SuperState.PRE_L3));

        stateTriggers
                .get(SuperState.READY_CORAL)
                .whileTrue(elevator.setGoal(Elevator.STOW))
                .and(preScoreReq)
                .and(() -> reefLevel == ReefLevel.L2)
                .onTrue(forceState(SuperState.PRE_L2));

        stateTriggers
                .get(SuperState.READY_CORAL)
                .whileTrue(elevator.setGoal(Elevator.STOW))
                .and(preScoreReq)
                .and(() -> reefLevel == ReefLevel.L1)
                .onTrue(forceState(SuperState.PRE_L1));

        stateTriggers
                .get(SuperState.PRE_L4)
                .whileTrue(elevator.setGoal(Elevator.L4))
                .and(elevator.atSetpoint())
                .and(scoreReq)
                .onTrue(forceState(SuperState.SCORE_CORAL));

        stateTriggers
                .get(SuperState.PRE_L3)
                .whileTrue(elevator.setGoal(Elevator.L3))
                .and(elevator.atSetpoint())
                .and(scoreReq)
                .onTrue(forceState(SuperState.SCORE_CORAL));

        stateTriggers
                .get(SuperState.PRE_L2)
                .whileTrue(elevator.setGoal(Elevator.L2))
                .and(elevator.atSetpoint())
                .and(scoreReq)
                .onTrue(forceState(SuperState.SCORE_CORAL));

        stateTriggers
                .get(SuperState.PRE_L1)
                .whileTrue(elevator.setGoal(Elevator.L1))
                .and(elevator.atSetpoint())
                .and(scoreReq)
                .onTrue(forceState(SuperState.SCORE_CORAL));

        stateTriggers
                .get(SuperState.PRE_L4)
                .or(stateTriggers.get(SuperState.PRE_L3))
                .and(elevatorSafeToMove)
                .and(stowReq)
                .onTrue(forceState(SuperState.READY_CORAL));

        stateTriggers
                .get(SuperState.PRE_L2)
                .or(stateTriggers.get(SuperState.PRE_L1))
                .and(stowReq)
                .onTrue(forceState(SuperState.READY_CORAL));

        stateTriggers.get(SuperState.SCORE_CORAL).and(scoreReq).whileTrue(intake.outake());

        stateTriggers
                .get(SuperState.SCORE_CORAL)
                .and(() -> lastState != SuperState.PRE_L3 && lastState != SuperState.PRE_L4)
                .and(stowReq)
                .onTrue(forceState(SuperState.IDLE));

        stateTriggers
                .get(SuperState.SCORE_CORAL)
                .and(elevatorSafeToMove)
                .and(stowReq)
                .onTrue(forceState(SuperState.IDLE));
    }

    public Trigger readyToScore() {
        return intake.holdingCoral();
    }

    public Trigger canIntake() {
        return stateTriggers.get(SuperState.INTAKE_CORAL);
    }

    public Command setLevel(ReefLevel reefLevel) {
        return runOnce(() -> this.reefLevel = reefLevel);
    }

    public Command forceState(SuperState nextState) {
        return runOnce(() -> {
                    System.out.println("Changing state to " + nextState);
                    this.lastState = state;
                    this.state = nextState;
                })
                .ignoringDisable(true);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Superstructure/Superstructure State", state);
    }
}
