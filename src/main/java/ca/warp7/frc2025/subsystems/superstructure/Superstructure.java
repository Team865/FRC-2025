package ca.warp7.frc2025.subsystems.superstructure;

import ca.warp7.frc2025.Constants.Elevator;
import ca.warp7.frc2025.FieldConstants.ReefLevel;
import ca.warp7.frc2025.subsystems.climber.ClimberSubsystem;
import ca.warp7.frc2025.subsystems.elevator.ElevatorSubsystem;
import ca.warp7.frc2025.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// fix barge controls

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
        SCORE_CORAL_L1,
        PRE_ALGAE_HIGH,
        PRE_ALGAE_LOW,
        CLEAR_ALGAE,
        BARGE,
        PRE_CLIMB,
        CLIMB
    }

    public static enum AlgaeLevel {
        HIGH,
        LOW,
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

    @AutoLogOutput(key = "Superstructure/Algae Level")
    private AlgaeLevel algaeLevel = AlgaeLevel.HIGH;

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
        // Idle
        stateTriggers
                .get(SuperState.IDLE)
                .onTrue(elevator.setGoal(Elevator.STOW))
                .and(() -> !isAlgaeLike())
                .onTrue(intake.setVoltsRoller(0));

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
                .get(SuperState.IDLE)
                .and(() -> lastState == SuperState.PRE_ALGAE_HIGH)
                .and(preScoreReq)
                .onTrue(forceState(SuperState.BARGE));

        stateTriggers
                .get(SuperState.BARGE)
                .onTrue(elevator.setGoal(Elevator.L4)
                        .andThen(new WaitCommand(0.5))
                        .andThen(intake.runVoltsRoller(-10).withTimeout(2)));

        stateTriggers.get(SuperState.BARGE).and(stowReq).onTrue(elevator.setGoal(Elevator.STOW));

        stateTriggers
                .get(SuperState.IDLE)
                .and(() -> lastState != SuperState.PRE_ALGAE_HIGH)
                .and(preScoreReq)
                .and(() -> algaeLevel == AlgaeLevel.HIGH)
                .onTrue(forceState(SuperState.PRE_ALGAE_HIGH));

        stateTriggers
                .get(SuperState.PRE_ALGAE_HIGH)
                .or(stateTriggers.get(SuperState.PRE_ALGAE_LOW))
                .and(stowReq)
                .onTrue(forceState(SuperState.IDLE));

        stateTriggers
                .get(SuperState.IDLE)
                .and(preScoreReq)
                .and(() -> algaeLevel == AlgaeLevel.LOW)
                .onTrue(forceState(SuperState.PRE_ALGAE_LOW));

        stateTriggers
                .get(SuperState.PRE_ALGAE_HIGH)
                .whileTrue(elevator.setGoal(Elevator.L2A))
                .whileTrue(intake.setTorque());
        // .whileTrue(intake.setVoltsRoller(-8));

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
                .onTrue(forceState(SuperState.SCORE_CORAL_L1));

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

        stateTriggers
                .get(SuperState.SCORE_CORAL_L1)
                .and(scoreReq)
                .whileTrue(intake.outake(-3))
                .and(intake.notHoldingCoral());

        stateTriggers.get(SuperState.SCORE_CORAL_L1).and(stowReq).onTrue(forceState(SuperState.IDLE));
    }

    public boolean isAlgaeLike() {
        return state == SuperState.PRE_ALGAE_LOW || state == SuperState.PRE_ALGAE_HIGH;
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

    public Command setAlgae(AlgaeLevel algaeLevel) {
        return runOnce(() -> this.algaeLevel = algaeLevel);
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
