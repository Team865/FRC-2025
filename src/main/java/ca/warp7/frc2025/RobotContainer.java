// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2025;

import ca.warp7.frc2025.Constants.Climber;
import ca.warp7.frc2025.Constants.Elevator;
import ca.warp7.frc2025.Constants.Intake;
import ca.warp7.frc2025.commands.DriveCommands;
import ca.warp7.frc2025.generated.TunerConstants;
import ca.warp7.frc2025.subsystems.Vision.VisionConstants;
import ca.warp7.frc2025.subsystems.Vision.VisionIO;
import ca.warp7.frc2025.subsystems.Vision.VisionIOLimelight;
import ca.warp7.frc2025.subsystems.Vision.VisionIOPhotonVisionSim;
import ca.warp7.frc2025.subsystems.Vision.VisionSubsystem;
import ca.warp7.frc2025.subsystems.climber.ClimberIO;
import ca.warp7.frc2025.subsystems.climber.ClimberIOTalonFX;
import ca.warp7.frc2025.subsystems.climber.ClimberSubsystem;
import ca.warp7.frc2025.subsystems.drive.DriveSubsystem;
import ca.warp7.frc2025.subsystems.drive.GyroIO;
import ca.warp7.frc2025.subsystems.drive.GyroIOPigeon2;
import ca.warp7.frc2025.subsystems.drive.ModuleIO;
import ca.warp7.frc2025.subsystems.drive.ModuleIOSim;
import ca.warp7.frc2025.subsystems.drive.ModuleIOTalonFX;
import ca.warp7.frc2025.subsystems.elevator.ElevatorIO;
import ca.warp7.frc2025.subsystems.elevator.ElevatorIOSim;
import ca.warp7.frc2025.subsystems.elevator.ElevatorIOTalonFX;
import ca.warp7.frc2025.subsystems.elevator.ElevatorSubsystem;
import ca.warp7.frc2025.subsystems.intake.IntakeSubsystem;
import ca.warp7.frc2025.subsystems.intake.ObjectDectionIO;
import ca.warp7.frc2025.subsystems.intake.ObjectDectionIOLaserCAN;
import ca.warp7.frc2025.subsystems.intake.RollersIO;
import ca.warp7.frc2025.subsystems.intake.RollersIOSim;
import ca.warp7.frc2025.subsystems.intake.RollersIOTalonFX;
import ca.warp7.frc2025.util.FieldConstantsHelper;
import ca.warp7.frc2025.util.VisionUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    // Subsystems
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ElevatorSubsystem elevator;
    private final ClimberSubsystem climber;
    private final VisionSubsystem vision;
    // private final LEDSubsystem leds;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    public enum ControlMode {
        MANUAL,
        ASSIST,
    }

    private ControlMode controlMode = ControlMode.ASSIST;

    private Trigger isManual = new Trigger(() -> controlMode == ControlMode.MANUAL);

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new DriveSubsystem(
                        new GyroIOPigeon2(
                                TunerConstants.DrivetrainConstants.Pigeon2Id,
                                TunerConstants.DrivetrainConstants.CANBusName),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                intake = new IntakeSubsystem(
                        new RollersIOTalonFX(2, "rio"),
                        new ObjectDectionIOLaserCAN(Intake.TOP_LASER_CAN),
                        new ObjectDectionIOLaserCAN(Intake.FRONT_LASER_CAN));

                climber = new ClimberSubsystem(
                        new ClimberIOTalonFX(Climber.PIVOT_ID, Climber.INTAKE_ID, Climber.Servo_PWM));

                elevator = new ElevatorSubsystem(new ElevatorIOTalonFX(11, 12));

                vision = new VisionSubsystem(
                        drive::addVisionMeasurement,
                        new VisionIOLimelight(VisionConstants.camera0Name, () -> drive.getRotation(), false),
                        new VisionIOLimelight(VisionConstants.camera1Name, () -> drive.getRotation(), false));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new DriveSubsystem(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                intake = new IntakeSubsystem(
                        new RollersIOSim(DCMotor.getKrakenX60(1), 1, 0.01),
                        new ObjectDectionIO() {},
                        new ObjectDectionIO() {});

                elevator = new ElevatorSubsystem(new ElevatorIOSim());

                climber = new ClimberSubsystem(new ClimberIO() {});

                vision = new VisionSubsystem(
                        drive::addVisionMeasurement,
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera0Name,
                                VisionConstants.robotToCamera0,
                                () -> drive.getPose(),
                                false),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera1Name,
                                VisionConstants.robotToCamera1,
                                () -> drive.getPose(),
                                false));
                break;
            case REPLAY:
            default:
                drive = new DriveSubsystem(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});

                intake = new IntakeSubsystem(new RollersIO() {}, new ObjectDectionIO() {}, new ObjectDectionIO() {});

                elevator = new ElevatorSubsystem(new ElevatorIO() {});

                climber = new ClimberSubsystem(new ClimberIO() {});

                vision = new VisionSubsystem(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

                // leds = new LEDSubsystem(0);
        }

        configureNamedCommands();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Autos", AutoBuilder.buildAutoChooser());

        if (Constants.tuningMode) {
            autoChooser.addOption(
                    "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
            autoChooser.addOption("Drive Wheel FF Characterization", DriveCommands.feedforwardCharacterization(drive));
            autoChooser.addOption(
                    "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption(
                    "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

            configureTuningBindings();
        } else {
            configureBindings();
        }
    }

    private void configureNamedCommands() {
        Command Stow = elevator.setGoal(Elevator.STOW).andThen(new WaitUntilCommand(elevator.atSetpointTrigger()));

        Command outakeCommand = new SequentialCommandGroup(new WaitUntilCommand(intake.bottomSensorTrigger()
                        .negate()
                        .and(intake.middleSensorTrigger().negate()))
                .deadlineFor(intake.runVoltsRoller(-6)));

        Command intakeCommand = new SequentialCommandGroup(
                        intake.runVoltsRoller(-8).until(intake.middleSensorTrigger()), elevator.setGoal(Elevator.STOW))
                .finallyDo(() -> intake.holding = true);

        NamedCommands.registerCommand("Stow", Stow);
        NamedCommands.registerCommand("L4", elevator.setGoal(Elevator.L4));
        NamedCommands.registerCommand("Intake", intakeCommand);
        NamedCommands.registerCommand("evIntake", elevator.setGoal(Elevator.STOW));
        NamedCommands.registerCommand("Outake", outakeCommand);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {
        Command driveCommand = DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX());

        Command intakeAngle = DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> FieldConstantsHelper.getClosestStation(drive.getPose())
                        .getRotation()
                        .rotateBy(Rotation2d.k180deg));

        Command slowModeToggle = drive.runOnce(() -> {
            if (drive.speedModifer != 1) {
                drive.speedModifer = 1;
            } else {
                drive.speedModifer = 0.5;
            }
        });

        // run intake motor until sensor
        Command intakeCommand = intake.runVoltsRoller(-4)
                .until(intake.bottomSensorTrigger())
                // leds.solidColorCommand(SparkColor.HOT_PINK),

                // leds.setBlinkingCmd(SparkColor.HOT_PINK, SparkColor.BLACK, 10))
                .finallyDo(() -> {
                    intake.holding = true;
                });

        Supplier<Command> outakeCommand = () -> new WaitUntilCommand(intake.bottomSensorTrigger()
                        .negate()
                        .and(intake.middleSensorTrigger().negate()))
                .deadlineFor(intake.runVoltsRoller(-4))
                .finallyDo(() -> {
                    intake.holding = false;
                    controller.setRumble(RumbleType.kBothRumble, 0.7);
                });

        Command stow =
                drive.setSpeedModifer(1).andThen(elevator.setGoal(Elevator.STOW).andThen(intake.setVoltsRoller(0)));

        Supplier<Command> align = () -> DriveCommands.poseLockDriveCommand(drive, () -> {
            return Optional.of(FieldConstantsHelper.faceToRobotPose(
                    FieldConstantsHelper.getclosestFace(drive.getPose()), drive.target == 0));
        });

        Trigger alignedTrigger = DriveCommands.isAligned(() -> drive.getPose(), () -> {
            if (vision.tags.size() > 0) {
                return Optional.of(VisionUtil.tagIdToRobotPose(vision.tags.get(0), drive.target == 0));
            } else {
                return Optional.empty();
            }
        });

        Trigger holdingCoral = intake.middleSensorTrigger();

        Trigger notHoldingCoral = holdingCoral.negate();

        Trigger atStow = elevator.atSetpointTrigger(Elevator.STOW);

        Command autoScoreL4 = elevator.setGoal(Elevator.L4)
                .andThen(align.get().until(alignedTrigger))
                .andThen(outakeCommand.get());

        Command autoScoreL3 = elevator.setGoal(Elevator.L3)
                .andThen(align.get().until(alignedTrigger))
                .andThen(outakeCommand.get());

        Command autoScoreL2 = elevator.setGoal(Elevator.L2)
                .andThen(align.get().until(alignedTrigger))
                .andThen(outakeCommand.get());

        Command algaeClearHigh = drive.setSpeedModifer(0.25)
                .andThen(elevator.setGoal(Elevator.L2A))
                .andThen(intake.setVoltsRoller(-4));

        Command algaeClearLow = drive.setSpeedModifer(0.25)
                .andThen(elevator.setGoal(Elevator.L1A))
                .andThen(intake.setVoltsRoller(-4));

        Command climberDown = climber.setPivotServoPosition(0).andThen(climber.setPivotPosition(Climber.CLIMB));

        Command climberStow = climber.setPivotPosition(1).andThen(climber.setPivotPosition(Climber.STOW));

        Command climb = climber.setPivotServoPosition(1)
                .andThen(climber.setPivotVoltage(10))
                .andThen(new WaitCommand(1))
                .andThen(climber.setPivotVoltage(0));

        drive.setDefaultCommand(driveCommand);

        controller.leftBumper().whileTrue(intakeAngle);

        controller.leftStick().onTrue(drive.zeroPose());
        controller.rightStick().onTrue(drive.zeroPose());

        controller.rightTrigger().onTrue(outakeCommand.get());

        controller
                .leftTrigger()
                .and(intake.middleSensorTrigger().negate())
                .and(elevator.atSetpointTrigger(Elevator.STOW))
                .onTrue(intakeCommand);

        // Coral & Algae set points
        controller.a().onTrue(stow);

        controller.y().whileTrue(autoScoreL4);

        controller.b().whileTrue(autoScoreL3);

        controller.x().whileTrue(autoScoreL2);

        // controller.b().and(atStow).and(notHoldingCoral).toggleOnTrue(a
        //
        // controller.x().and(atStow).and(notHoldingCoral).onTrue(algaeClearLow);

        controller.povLeft().onTrue(drive.runOnce(() -> drive.target = 0));
        controller.povRight().onTrue(drive.runOnce(() -> drive.target = 1));

        controller.povDown().onTrue(climberDown);

        controller.povUp().onTrue(climb);

        controller.back().onTrue(climberStow);

        controller.rightBumper().whileTrue(intake.runVoltsRoller(4));

        controller.start().toggleOnTrue(slowModeToggle);
    }

    private void configureTuningBindings() {}

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
