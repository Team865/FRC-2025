// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2025;

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
import ca.warp7.frc2025.util.ClosestHPStation;
import ca.warp7.frc2025.util.VisionUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    // Subsystems
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ElevatorSubsystem elevator;
    private final ClimberSubsystem climber;
    private final VisionSubsystem vision;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

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

                climber = new ClimberSubsystem(new ClimberIOTalonFX(62, 52));

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
                                VisionConstants.camera0Name, VisionConstants.robotToCamera0, () -> drive.getPose()),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera1Name, VisionConstants.robotToCamera1, () -> drive.getPose()));
                break;
            case REPLAY:
            default:
                drive = new DriveSubsystem(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});

                intake = new IntakeSubsystem(new RollersIO() {}, new ObjectDectionIO() {}, new ObjectDectionIO() {});

                elevator = new ElevatorSubsystem(new ElevatorIO() {});

                climber = new ClimberSubsystem(new ClimberIO() {});

                vision = new VisionSubsystem(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
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
        Trigger Lockout = new Trigger(() -> vision.getPoseObv(drive.target) != null
                && vision.getPoseObv(drive.target).averageTagDistance() > 0.3);

        Logger.recordOutput("Lockout", Lockout);

        Command Stow = elevator.setGoal(Elevator.STOW).andThen(new WaitUntilCommand(elevator.atSetpointTrigger()));

        NamedCommands.registerCommand("Stow", Stow);

        Command align = DriveCommands.reefAlign(
                drive,
                () -> vision.getTarget(drive.target).tx(),
                () -> vision.getTarget(drive.target).ty(),
                () -> VisionConstants.tx[drive.target],
                () -> vision.getTagID(drive.target)
                        .map((id) -> VisionUtil.validId(id, drive.getRotation())
                                ? VisionConstants.getTy(id, drive.target)
                                : Rotation2d.fromDegrees(0))
                        .orElse(Rotation2d.fromDegrees(0)),
                () -> Optional.empty());

        Trigger reefAlignTrigger = DriveCommands.isReefAlignedTigger(
                () -> vision.getTarget(drive.target).tx(),
                () -> vision.getTarget(drive.target).ty(),
                () -> VisionConstants.tx[drive.target],
                () -> vision.getTagID(drive.target)
                        .map((id) -> VisionUtil.validId(id, drive.getRotation())
                                ? VisionConstants.getTy(id, drive.target)
                                : Rotation2d.fromDegrees(0))
                        .orElse(Rotation2d.fromDegrees(0)));
        //
        SequentialCommandGroup outakeCommand = new SequentialCommandGroup(
                new WaitCommand(1).deadlineFor(intake.runVoltsRoller(-4)), intake.setHolding(false));
        //
        Command autoScore = new SequentialCommandGroup(
                new WaitUntilCommand(Lockout),
                elevator.setGoal(Elevator.L4),
                new WaitUntilCommand(elevator.atSetpointTrigger()),
                align.until(reefAlignTrigger),
                drive.runOnce(() -> vision.getTagID(drive.target)
                        .ifPresent((id) -> drive.setPose(VisionUtil.tagIdToRobotPose(id, drive.target == 0)))),
                outakeCommand);

        NamedCommands.registerCommand("autoScore", autoScore);

        SequentialCommandGroup intakeCommand = new SequentialCommandGroup(
                elevator.setGoal(Elevator.INTAKE),
                new WaitUntilCommand(elevator.atSetpointTrigger()),
                intake.runVoltsRoller(-4).until(intake.topSensorTrigger()),
                intake.runVoltsRoller(4).until(intake.topSensorTrigger().negate()),
                elevator.setGoal(Elevator.STOW),
                new WaitUntilCommand(elevator.atSetpointTrigger()),
                intake.setHolding(true));

        NamedCommands.registerCommand("scoreRight", drive.runOnce(() -> drive.target = 1));
        NamedCommands.registerCommand("scoreLeft", drive.runOnce(() -> drive.target = 0));
        NamedCommands.registerCommand("Intake", intakeCommand);
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
                () -> ClosestHPStation.getClosestStation(drive.getPose())
                        .getRotation()
                        .rotateBy(Rotation2d.k180deg));

        drive.setDefaultCommand(driveCommand);

        controller.leftBumper().whileTrue(intakeAngle);

        controller.leftStick().onTrue(drive.zeroPose());
        controller.rightStick().onTrue(drive.zeroPose());

        // run intake motor until sensor
        SequentialCommandGroup intakeCommand = new SequentialCommandGroup(
                elevator.setGoal(Elevator.INTAKE),
                new WaitUntilCommand(elevator.atSetpointTrigger()),
                intake.runVoltsRoller(-4).until(intake.topSensorTrigger()),
                intake.runVoltsRoller(1).until(intake.topSensorTrigger().negate()),
                elevator.setGoal(Elevator.STOW),
                new WaitUntilCommand(elevator.atSetpointTrigger()),
                intake.setHolding(true));

        SequentialCommandGroup outakeCommand = new SequentialCommandGroup(
                new WaitCommand(0.5).deadlineFor(intake.runVoltsRoller(-4)), intake.setHolding(false));

        controller.rightTrigger().and(intake.holdingTrigger()).onTrue(outakeCommand);
        controller
                .leftTrigger()
                .and(intake.holdingTrigger().negate())
                .and(elevator.atSetpointTrigger(Elevator.STOW))
                .onTrue(intakeCommand);

        controller.start().toggleOnTrue(drive.runOnce(() -> {
            if (drive.speedModifer != 1) {
                drive.speedModifer = 1;
            } else {
                drive.speedModifer = 0.5;
            }
        }));

        Trigger L4 = new Trigger(() -> vision.getPoseObv(drive.target) != null
                && vision.getPoseObv(drive.target).averageTagDistance() > 0.45);

        controller
                .a()
                // .and(L4)
                .onTrue(drive.runOnce(() -> drive.speedModifer = 1).andThen(elevator.setGoal(Elevator.STOW)));

        controller.y().onTrue(drive.runOnce(() -> drive.speedModifer = 1).andThen(elevator.setGoal(Elevator.INTAKE)));

        controller.x().onTrue(drive.runOnce(() -> drive.speedModifer = 0.25).andThen(elevator.setGoal(Elevator.L2)));

        controller.b().onTrue(drive.runOnce(() -> drive.speedModifer = 0.25).andThen(elevator.setGoal(Elevator.L3)));

        BooleanSupplier Lockout = () -> vision.getPoseObv(drive.target) != null
                && vision.getPoseObv(drive.target).averageTagDistance() > 0.45;

        Command align = DriveCommands.reefAlign(
                drive,
                () -> vision.getTarget(drive.target).tx(),
                () -> vision.getTarget(drive.target).ty(),
                () -> VisionConstants.tx[drive.target],
                () -> vision.getTagID(drive.target)
                        .map((id) -> VisionUtil.validId(id, drive.getRotation())
                                ? VisionConstants.getTy(id, drive.target)
                                : Rotation2d.fromDegrees(0))
                        .orElse(Rotation2d.fromDegrees(0)),
                () -> vision.getTagID(drive.target)
                        .flatMap((id) -> VisionUtil.validId(id, drive.getRotation())
                                ? VisionConstants.aprilTagLayout
                                        .getTagPose(id)
                                        .flatMap((pose) ->
                                                Optional.of(pose.getRotation().toRotation2d()))
                                : Optional.empty()));

        Trigger reefAlignTrigger = DriveCommands.isReefAlignedTigger(
                () -> vision.getTarget(drive.target).tx(),
                () -> vision.getTarget(drive.target).ty(),
                () -> VisionConstants.tx[drive.target],
                () -> vision.getTagID(drive.target)
                        .map((id) -> VisionUtil.validId(id, drive.getRotation())
                                ? VisionConstants.getTy(id, drive.target)
                                : Rotation2d.fromDegrees(0))
                        .orElse(Rotation2d.fromDegrees(0)));

        Command autoScore = new SequentialCommandGroup(
                new WaitUntilCommand(Lockout),
                elevator.setGoal(Elevator.L4),
                align.until(reefAlignTrigger),
                drive.runOnce(() -> vision.getTagID(drive.target)
                        .ifPresent((id) -> drive.setPose(VisionUtil.tagIdToRobotPose(id, drive.target == 1)))),
                outakeCommand);

        CommandScheduler.getInstance().removeComposedCommand(outakeCommand);
        CommandScheduler.getInstance().removeComposedCommand(align);

        controller.povLeft().onTrue(drive.runOnce(() -> drive.target = 0));
        controller.povRight().onTrue(drive.runOnce(() -> drive.target = 1));

        controller.y().whileTrue(autoScore);
    }

    private void configureTuningBindings() {}

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
