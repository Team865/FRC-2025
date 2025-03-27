// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2025;

import ca.warp7.frc2025.Constants.Climber;
import ca.warp7.frc2025.Constants.Elevator;
import ca.warp7.frc2025.Constants.Intake;
import ca.warp7.frc2025.FieldConstants.ReefLevel;
import ca.warp7.frc2025.commands.DriveCommands;
import ca.warp7.frc2025.commands.DriveToPose;
import ca.warp7.frc2025.generated.TunerConstants;
import ca.warp7.frc2025.subsystems.Vision.VisionConstants;
import ca.warp7.frc2025.subsystems.Vision.VisionIO;
import ca.warp7.frc2025.subsystems.Vision.VisionIOLimelight;
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
import ca.warp7.frc2025.subsystems.leds.LEDSubsystem;
import ca.warp7.frc2025.subsystems.superstructure.Superstructure;
import ca.warp7.frc2025.subsystems.superstructure.Superstructure.AlgaeLevel;
import ca.warp7.frc2025.subsystems.superstructure.Superstructure.SuperState;
import ca.warp7.frc2025.util.FieldConstantsHelper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    // Subsystems
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ElevatorSubsystem elevator;
    private final ClimberSubsystem climber;
    private final VisionSubsystem vision;
    private final LEDSubsystem leds;

    // Superstructure
    private final Superstructure superstructure;

    // Controller
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

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

                vision = new VisionSubsystem(drive::addVisionMeasurement);
                // new VisionIOPhotonVisionSim(
                //         VisionConstants.camera0Name,
                //         VisionConstants.robotToCamera0,
                //         () -> drive.getPose(),
                //         false),
                // new VisionIOPhotonVisionSim(
                //         VisionConstants.camera1Name,
                //         VisionConstants.robotToCamera1,
                //         () -> drive.getPose(),
                //         false));
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

        Trigger alignedTrigger = DriveCommands.isAligned(() -> drive.getPose(), () -> {
            Pose2d pose2d = FieldConstantsHelper.faceToRobotPose(
                    FieldConstantsHelper.getclosestFace(drive.getPose()), drive.target == 0);
            Logger.recordOutput("alignedPose", pose2d);
            return Optional.of(pose2d);
        });

        Trigger toFarForExtend = new Trigger(() -> {
            double distance =
                    FieldConstantsHelper.lengthFromCenterOfReef(drive.getPose()).magnitude();

            return 3 <= distance;
        });

        Trigger toCloseForExtension = new Trigger(() -> {
            double distance =
                    FieldConstantsHelper.lengthFromCenterOfReef(drive.getPose()).magnitude();

            return distance <= 1.5;
        });

        superstructure = new Superstructure(
                elevator,
                intake,
                climber,
                driveController.y().or(driveController.x()),
                driveController.rightTrigger(),
                alignedTrigger,
                driveController.a(),
                driveController.povDown(),
                driveController.povUp(),
                toCloseForExtension,
                toFarForExtend);

        leds = new LEDSubsystem(2);
        leds.setToDefault();
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

        } else {
        }
        configureBindings();
        // configureTuningBindings();
    }

    private void configureNamedCommands() {
        Command Stow = elevator.setGoal(Elevator.STOW).andThen(new WaitUntilCommand(elevator.atSetpointTrigger()));

        NamedCommands.registerCommand("Stow", Stow);
        NamedCommands.registerCommand("L4", elevator.setGoal(Elevator.L4));
        NamedCommands.registerCommand("Intake", intake.intake());
        NamedCommands.registerCommand("Outake", intake.outake());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {
        Command driveCommand = DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> -driveController.getRightX());

        Command intakeAngle = DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> FieldConstantsHelper.getClosestStation(drive.getPose())
                        .getRotation()
                        .rotateBy(Rotation2d.k180deg));

        Command driveToHumanPlayer = new DriveToPose(
                drive,
                () -> FieldConstantsHelper.stationToRobot(FieldConstantsHelper.getClosestStation(drive.getPose())));

        Command driveReefAngleCenter = DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> FieldConstantsHelper.getAngleToReefCenter(drive.getPose()));

        Supplier<Pose2d> alignedGoal = () -> {
            Pose2d pose2d = FieldConstantsHelper.faceToRobotPose(
                    FieldConstantsHelper.getclosestFace(drive.getPose()), drive.target == 0);
            Logger.recordOutput("alignedPoseGoal", pose2d);
            return pose2d;
        };

        Trigger alignedTrigger = DriveCommands.isAligned(() -> drive.getPose(), () -> {
            Pose2d pose2d = FieldConstantsHelper.faceToRobotPose(
                    FieldConstantsHelper.getclosestFace(drive.getPose()), drive.target == 0);
            Logger.recordOutput("alignedPose", pose2d);
            return Optional.of(pose2d);
        });

        // Supplier<Command> align = () -> DriveCommands.poseLockDriveCommand(drive, () -> {
        //     Pose2d pose2d = FieldConstantsHelper.faceToRobotPose(
        //             FieldConstantsHelper.getclosestFace(drive.getPose()), drive.target == 0);
        //     Logger.recordOutput("alignedPoseGoal", pose2d);
        //     return Optional.of(pose2d);
        // });

        Supplier<Command> align = () -> new DriveToPose(drive, () -> {
            Pose2d pose2d = FieldConstantsHelper.faceToRobotPose(
                    FieldConstantsHelper.getclosestFace(drive.getPose()), drive.target == 0);
            Logger.recordOutput("alignedPoseGoal", pose2d);
            return pose2d;
        });

        Command scoreLeft = drive.runOnce(() -> drive.target = 0);
        Command scoreRight = drive.runOnce(() -> drive.target = 1);

        operatorController.y().onTrue(superstructure.setLevel(ReefLevel.L4));
        operatorController.x().onTrue(superstructure.setLevel(ReefLevel.L3));
        operatorController.b().onTrue(superstructure.setLevel(ReefLevel.L2));
        operatorController.a().onTrue(superstructure.setLevel(ReefLevel.L1));

        driveController.povUp().onTrue(superstructure.setLevel(ReefLevel.L4));
        driveController.povRight().onTrue(superstructure.setLevel(ReefLevel.L3));
        driveController.povLeft().onTrue(superstructure.setLevel(ReefLevel.L2));
        driveController.povDown().onTrue(superstructure.setLevel(ReefLevel.L1));

        operatorController.povLeft().onTrue(scoreLeft);
        operatorController.povRight().onTrue(scoreRight);

        operatorController.povUp().onTrue(superstructure.setAlgae(AlgaeLevel.HIGH));
        operatorController.povDown().onTrue(superstructure.setAlgae(AlgaeLevel.LOW));

        operatorController.back().onTrue(superstructure.forceState(SuperState.IDLE));

        // operatorController.povUp().onTrue(elevator.set)

        // driveController.x().onTrue(scoreLeft);
        // driveController.b().onTrue(scoreRight);

        drive.setDefaultCommand(driveCommand);

        driveController
                .rightTrigger()
                .and(superstructure.readyToScore())
                .whileTrue(align.get().until(alignedTrigger));

        // driveController.x().and(superstructure.canIntake()).whileTrue(intakeAngle);
        driveController.y().and(superstructure.canIntake()).whileTrue(driveToHumanPlayer);
        driveController.x().whileTrue(intakeAngle);

        driveController.povRight().whileTrue(intake.runVoltsRoller(-10));
        driveController.povLeft().whileTrue(intake.runVoltsRoller(4));

        driveController.povRight().whileTrue(intake.setTorque());

        driveController.start().onTrue(drive.zeroPose());
        driveController.back().onTrue(superstructure.forceState(SuperState.IDLE));

        driveController.leftTrigger().toggleOnTrue(driveReefAngleCenter).toggleOnFalse(driveCommand);

        // Fix elevator thing
    }

    private void configureTuningBindings() {
        Supplier<Command> align = () -> new DriveToPose(drive, () -> {
            Pose2d pose2d = FieldConstantsHelper.faceToRobotPose(
                    FieldConstantsHelper.getclosestFace(drive.getPose()), drive.target == 0);
            Logger.recordOutput("alignedPoseGoal", pose2d);
            return pose2d;
        });

        Command driveCommand = DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> -driveController.getRightX());

        drive.setDefaultCommand(driveCommand);

        driveController.rightTrigger().whileTrue(align.get());
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
