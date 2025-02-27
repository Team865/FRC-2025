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
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
                        new VisionIOLimelight(VisionConstants.camera0Name, () -> drive.getRotation()),
                        new VisionIOLimelight(VisionConstants.camera1Name, () -> drive.getRotation()));
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

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX(),
                () -> drive.speedModifer));

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
                intake.runVoltsRoller(-4).until(intake.topSensorTrigger().negate()), intake.setHolding(false));

        // controller
        //         .leftTrigger()
        //         .and(intake.frontSensorTrigger()
        //                 .negate()
        //                 .and(intake.topSensorTrigger().negate()))
        //         .onTrue(intakeCommand);

        controller.leftTrigger().and(intake.holdingTrigger()).onTrue(outakeCommand);
        controller.leftTrigger().and(intake.holdingTrigger().negate()).onTrue(intakeCommand);

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
                .b()
                .and(L4)
                .onTrue(drive.runOnce(() -> drive.speedModifer = 0.25).andThen(elevator.setGoal(Elevator.L4)));
        controller.a().onTrue(drive.runOnce(() -> drive.speedModifer = 1).andThen(elevator.setGoal(Elevator.STOW)));
        controller.y().onTrue(drive.runOnce(() -> drive.speedModifer = 1).andThen(elevator.setGoal(Elevator.INTAKE)));
        controller.x().onTrue(drive.runOnce(() -> drive.speedModifer = 0.25).andThen(elevator.setGoal(Elevator.L3)));
        controller
                .leftBumper()
                .onTrue(drive.runOnce(() -> drive.speedModifer = 0.25).andThen(elevator.setGoal(Elevator.L2)));
        Command align = DriveCommands.reefAlign(
                drive,
                () -> vision.getTarget(drive.target).tx(),
                () -> vision.getTarget(drive.target).ty(),
                () -> VisionConstants.tx[drive.target],
                () -> VisionConstants.ty[drive.target],
                () -> vision.tag()
                        .map((pose2d) -> pose2d.rotateBy(Rotation2d.k180deg))
                        .orElse(new Pose2d()));
        controller.povLeft().onTrue(drive.runOnce(() -> drive.target = 1));
        controller.povRight().onTrue(drive.runOnce(() -> drive.target = 0));
        controller.povDown().whileTrue(align);
    }

    private void configureTuningBindings() {
        controller.leftStick().onTrue(drive.zeroPose());
        controller.rightStick().onTrue(drive.zeroPose());

        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX(),
                () -> drive.speedModifer));

        Command align = DriveCommands.reefAlign(
                drive,
                () -> vision.getTarget(drive.target).tx(),
                () -> vision.getTarget(drive.target).ty(),
                () -> VisionConstants.tx[drive.target],
                () -> VisionConstants.ty[drive.target],
                () -> vision.getTag(drive.target).orElse(new Pose2d()));

        controller.povLeft().onTrue(drive.runOnce(() -> drive.target = 1));
        controller.povRight().onTrue(drive.runOnce(() -> drive.target = 0));
        controller.a().whileTrue(align);
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
