package ca.warp7.frc2025.subsystems.superstructure;

import ca.warp7.frc2025.Constants.Elevator;
import ca.warp7.frc2025.FieldConstants.ReefLevel;
import ca.warp7.frc2025.subsystems.climber.ClimberSubsystem;
import ca.warp7.frc2025.subsystems.elevator.ElevatorSubsystem;
import ca.warp7.frc2025.subsystems.intake.IntakeSubsystem;
import ca.warp7.frc2025.subsystems.leds.LEDSubsystem;
import ca.warp7.frc2025.subsystems.leds.LEDSubsystem.SparkColor;
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
        INTAKE_ALGAE,
        READY_ALGAE,
        SPIT_ALGAE,
        BARGE,
        PRE_CLIMB,
        CLIMB
    }

    public static enum AlgaeLevel {
        HIGH,
        LOW,
    }

    public static enum AlgaeTarget {
        BARGE,
        PROCESSOR,
    }

    @AutoLogOutput(key = "Superstructure/Intake Req")
    private final Trigger intakeReq;

    @AutoLogOutput(key = "Superstructure/Pre Score Req")
    private final Trigger preScoreReq;

    @AutoLogOutput(key = "Superstructure/Score Req")
    private final Trigger scoreReq;

    @AutoLogOutput(key = "Superstructure/Aligned To Algae Req")
    private final Trigger scoreAlgaeReq;

    @AutoLogOutput(key = "Superstructure/Stow Req")
    private final Trigger stowReq;

    @AutoLogOutput(key = "Superstructure/Pre Climber Req")
    private final Trigger preClimberReq;

    @AutoLogOutput(key = "Superstructure/Climber Req")
    private final Trigger climberReq;

    @AutoLogOutput(key = "Superstructure/Elev Too Close")
    private final Trigger elevatorTooClose;

    @AutoLogOutput(key = "Superstructure/Elev Too Far")
    private final Trigger elevatorTooFar;

    @AutoLogOutput(key = "Superstructure/Reef Level")
    private ReefLevel reefLevel = ReefLevel.L4;

    @AutoLogOutput(key = "Superstructure/Algae Level")
    private AlgaeLevel algaeLevel = AlgaeLevel.HIGH;

    @AutoLogOutput(key = "Superstructure/Algae Target")
    private AlgaeTarget algaeTarget = AlgaeTarget.PROCESSOR;

    private SuperState lastState = SuperState.IDLE;
    private SuperState state = SuperState.IDLE;
    private Map<SuperState, Trigger> stateTriggers = new HashMap<SuperState, Trigger>();

    private final ElevatorSubsystem elevator;
    private final IntakeSubsystem intake;
    private final ClimberSubsystem climber;
    private final LEDSubsystem leds;

    private Command slowMode;
    private Command normalMode;

    public Superstructure(
            ElevatorSubsystem elevator,
            IntakeSubsystem intake,
            ClimberSubsystem climber,
            LEDSubsystem leds,
            Trigger intakeReq,
            Trigger preScoreReq,
            Trigger scoreReq,
            Trigger scoreAlgaeReq,
            Trigger stowReq,
            Trigger preClimberReq,
            Trigger climberReq,
            Trigger elevatorTooClose,
            Trigger elevatorTooFar,
            Command slowMode,
            Command normalMode) {
        this.elevator = elevator;
        this.intake = intake;
        this.climber = climber;
        this.leds = leds;

        this.intakeReq = intakeReq;
        this.preScoreReq = preScoreReq;
        this.scoreReq = scoreReq;
        this.scoreAlgaeReq = scoreAlgaeReq;
        this.stowReq = stowReq;
        this.preClimberReq = preClimberReq;
        this.climberReq = climberReq;
        this.elevatorTooClose = elevatorTooClose;
        this.elevatorTooFar = elevatorTooFar;

        this.slowMode = slowMode;
        this.normalMode = normalMode;

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
                .onTrue(leds.setToDefault())
                .and(() -> !isAlgaeLike())
                .onTrue(intake.setVoltsRoller(0));

        stateTriggers
                .get(SuperState.IDLE)
                .or(stateTriggers.get(SuperState.READY_CORAL))
                .and(preClimberReq)
                // .onTrue(slowMode)
                .whileTrue(climber.down())
                .and(climber.atSetpointTrigger())
                .onTrue(forceState(SuperState.CLIMB));

        stateTriggers.get(SuperState.CLIMB).and(climberReq).whileTrue(climber.climb());

        stateTriggers.get(SuperState.IDLE).and(intakeReq).onTrue(forceState(SuperState.INTAKE_CORAL));

        stateTriggers
                .get(SuperState.READY_ALGAE)
                .whileTrue(intake.setTorque())
                .whileTrue(elevator.setGoal(Elevator.STOW))
                .and(preScoreReq)
                .and(() -> algaeTarget == AlgaeTarget.PROCESSOR)
                .onTrue(forceState(SuperState.SPIT_ALGAE));

        stateTriggers
                .get(SuperState.SPIT_ALGAE)
                .onTrue(intake.runVoltsRoller(-10).withTimeout(2.0).andThen(forceState(SuperState.IDLE)));

        stateTriggers
                .get(SuperState.READY_ALGAE)
                .whileTrue(intake.setTorque())
                .whileTrue(elevator.setGoal(Elevator.STOW))
                .and(preScoreReq)
                .and(() -> algaeTarget == AlgaeTarget.BARGE)
                .onTrue(forceState(SuperState.BARGE));

        stateTriggers
                .get(SuperState.BARGE)
                .onTrue(elevator.setGoal(Elevator.L4)
                        .andThen(new WaitCommand(0.5))
                        .andThen(intake.runVoltsRoller(-10).withTimeout(2))
                        .andThen(forceState(SuperState.IDLE)));

        stateTriggers
                .get(SuperState.IDLE)
                .and(preScoreReq)
                .and(elevatorTooClose.negate())
                .and(() -> algaeLevel == AlgaeLevel.HIGH)
                .onTrue(forceState(SuperState.PRE_ALGAE_HIGH));

        // stateTriggers
        //         .get(SuperState.PRE_ALGAE_HIGH)
        //         .or(stateTriggers.get(SuperState.PRE_ALGAE_LOW))
        //         .and(stowReq)
        //         .onTrue(forceState(SuperState.READY_ALGAE));

        stateTriggers
                .get(SuperState.IDLE)
                .and(preScoreReq)
                .and(() -> algaeLevel == AlgaeLevel.LOW)
                .onTrue(forceState(SuperState.PRE_ALGAE_LOW));

        stateTriggers
                .get(SuperState.PRE_ALGAE_HIGH)
                .and(elevatorTooFar.negate())
                .whileTrue(elevator.setGoal(Elevator.L2A))
                .onTrue(forceState(SuperState.INTAKE_ALGAE));

        stateTriggers
                .get(SuperState.PRE_ALGAE_LOW)
                .and(elevatorTooFar.negate())
                .whileTrue(elevator.setGoal(Elevator.L1A))
                .onTrue(forceState(SuperState.INTAKE_ALGAE));

        stateTriggers.get(SuperState.INTAKE_ALGAE).and(scoreAlgaeReq).whileTrue(intake.setTorque());

        stateTriggers
                .get(SuperState.INTAKE_ALGAE)
                .and(elevatorTooClose.negate())
                .and(preScoreReq.negate())
                .onTrue(forceState(SuperState.READY_ALGAE));

        stateTriggers
                .get(SuperState.INTAKE_CORAL)
                .whileTrue(intake.intake())
                .whileTrue(leds.setBlinkingCmd(SparkColor.GREEN, SparkColor.BLACK, 5))
                .and(intake.holdingCoral())
                .onTrue(forceState(SuperState.READY_CORAL));

        stateTriggers
                .get(SuperState.READY_CORAL)
                .whileTrue(leds.setBlinkingCmd(SparkColor.LIME, SparkColor.BLACK, 20)
                        .withTimeout(0.75)
                        .andThen(leds.setToDefault()));
        stateTriggers
                .get(SuperState.READY_CORAL)
                .whileTrue(elevator.setGoal(Elevator.STOW))
                .and(elevatorTooFar.negate())
                .and(() -> reefLevel == ReefLevel.L4)
                .onTrue(forceState(SuperState.PRE_L4));

        stateTriggers
                .get(SuperState.READY_CORAL)
                .whileTrue(elevator.setGoal(Elevator.STOW))
                .and(elevatorTooFar.negate())
                .and(() -> reefLevel == ReefLevel.L3)
                .onTrue(forceState(SuperState.PRE_L3));

        stateTriggers
                .get(SuperState.READY_CORAL)
                .whileTrue(elevator.setGoal(Elevator.STOW))
                .and(scoreReq)
                .and(() -> reefLevel == ReefLevel.L2)
                .onTrue(forceState(SuperState.PRE_L2));

        stateTriggers
                .get(SuperState.READY_CORAL)
                .whileTrue(elevator.setGoal(Elevator.STOW))
                .and(scoreReq)
                .and(() -> reefLevel == ReefLevel.L1)
                .onTrue(forceState(SuperState.PRE_L1));

        stateTriggers
                .get(SuperState.PRE_L4)
                .and(preScoreReq)
                .whileTrue(elevator.setGoal(Elevator.L4))
                .and(elevator.atSetpoint())
                .and(scoreReq)
                .onTrue(forceState(SuperState.SCORE_CORAL));

        stateTriggers
                .get(SuperState.PRE_L3)
                .and(preScoreReq)
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
                .and(elevatorTooClose.negate())
                .and(stowReq)
                .onTrue(forceState(SuperState.READY_CORAL));

        stateTriggers
                .get(SuperState.PRE_L2)
                .or(stateTriggers.get(SuperState.PRE_L1))
                .and(stowReq)
                .onTrue(forceState(SuperState.READY_CORAL));

        stateTriggers
                .get(SuperState.SCORE_CORAL)
                .and(scoreReq)
                .onTrue(leds.setBlinkingCmd(SparkColor.GREEN, SparkColor.BLACK, 5))
                .whileTrue(intake.outake())
                .and(intake.notHoldingCoral())
                .onTrue(leds.setBlinkingCmd(SparkColor.GREEN, SparkColor.BLACK, 20));

        stateTriggers
                .get(SuperState.SCORE_CORAL)
                .and(() -> lastState == SuperState.PRE_L3 || lastState == SuperState.PRE_L4)
                .and(elevatorTooClose.negate())
                .onTrue(forceState(SuperState.IDLE));

        stateTriggers
                .get(SuperState.SCORE_CORAL)
                .and(() -> lastState != SuperState.PRE_L3 && lastState != SuperState.PRE_L4)
                .onTrue(forceState(SuperState.IDLE));

        stateTriggers
                .get(SuperState.SCORE_CORAL_L1)
                .and(elevator.atSetpoint())
                .and(scoreReq)
                .whileTrue(intake.outake(-3));

        stateTriggers
                .get(SuperState.SCORE_CORAL_L1)
                .and(intake.notHoldingCoral())
                .onTrue(forceState(SuperState.IDLE));
    }

    public boolean isAlgaeLike() {
        return state == SuperState.PRE_ALGAE_LOW || state == SuperState.PRE_ALGAE_HIGH;
    }

    public Trigger readyToScore() {
        return intake.holdingCoral();
    }

    public Trigger holdingAlgae() {
        return new Trigger(() -> state == SuperState.READY_ALGAE || state == SuperState.SPIT_ALGAE);
    }

    @AutoLogOutput(key = "Superstructure/Intake Coral Trigger")
    public Trigger canIntake() {
        return stateTriggers.get(SuperState.INTAKE_CORAL).or(stateTriggers.get(SuperState.IDLE));
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
