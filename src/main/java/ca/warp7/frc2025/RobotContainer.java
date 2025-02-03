// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2025;

import static edu.wpi.first.units.Units.Inches;

import ca.warp7.frc2025.commands.DriveCommands;
import ca.warp7.frc2025.subsystems.Climber.ClimberIOSim;
import ca.warp7.frc2025.subsystems.Climber.ClimberIOTalonFX;
import ca.warp7.frc2025.subsystems.Climber.ClimberSubsystem;
import ca.warp7.frc2025.subsystems.drive.DriveSubsystem;
import ca.warp7.frc2025.subsystems.drive.GyroIO;
import ca.warp7.frc2025.subsystems.drive.GyroIOPigeon2;
import ca.warp7.frc2025.subsystems.drive.ModuleIO;
import ca.warp7.frc2025.subsystems.drive.ModuleIOSim;
import ca.warp7.frc2025.subsystems.drive.ModuleIOTalonFX;
import ca.warp7.frc2025.subsystems.elevator.ElevatorIOSim;
import ca.warp7.frc2025.subsystems.elevator.ElevatorSubsystem;
import ca.warp7.frc2025.subsystems.generated.TunerConstants;
import ca.warp7.frc2025.subsystems.intake.IntakeSubsystem;
import ca.warp7.frc2025.subsystems.intake.ObjectDectionIO;
import ca.warp7.frc2025.subsystems.intake.ObjectDectionIOLaserCAN;
import ca.warp7.frc2025.subsystems.intake.RollersIO;
import ca.warp7.frc2025.subsystems.intake.RollersIOSim;
import ca.warp7.frc2025.subsystems.intake.RollersIOTalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    // Subsystems
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private ElevatorSubsystem elevator = null;
    private final ClimberSubsystem climber;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     *
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REPLAY:
                drive = new DriveSubsystem(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                intake = new IntakeSubsystem(new RollersIO() {}, new ObjectDectionIO() {}, new ObjectDectionIO() {});
                climber = new ClimberSubsystem(new ClimberIOSim());
                break;
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new DriveSubsystem(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                intake = new IntakeSubsystem(
                        new RollersIOTalonFX(1, "rio"),
                        new ObjectDectionIOLaserCAN(2, "Top"),
                        new ObjectDectionIOLaserCAN(3, "Front"));
                climber = new ClimberSubsystem(new ClimberIOTalonFX());
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
                break;
            default:
                drive = null;
                intake = null;
                climber = new ClimberSubsystem(new ClimberIOSim());
                break;
            default:
                drive = null;
                climber = null;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Autos", AutoBuilder.buildAutoChooser());

        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        configureBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        controller.rightStick().onTrue(drive.zeroPose());

        controller
                .a()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> Rotation2d.fromDegrees(90)));
        // run intake motor until sensor
        SequentialCommandGroup intakeCommand = new SequentialCommandGroup(
                intake.runVoltsRoller(-4).until(intake.topSensorTrigger()),
                intake.runVoltsRoller(1).until(intake.topSensorTrigger().negate()));

        controller
                .leftTrigger()
                .and(intake.frontSensorTrigger()
                        .negate()
                        .and(intake.topSensorTrigger().negate()))
                .onTrue(intakeCommand);

        controller
                .leftTrigger()
                .and(intake.frontSensorTrigger())
                .onTrue(intake.runVoltsRoller(-4)
                        .until(intake.topSensorTrigger()
                                .negate()
                                .and(intake.frontSensorTrigger().negate())));

        controller.x().onTrue(elevator.setGoal(Inches.of(15)));
        controller.y().onTrue(elevator.setGoal(Inches.of(28)));
        controller.b().onTrue(elevator.setGoal(Inches.of(0)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
