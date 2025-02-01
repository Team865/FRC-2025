// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2025;

import ca.warp7.frc2025.commands.DriveCommands;
import ca.warp7.frc2025.subsystems.drive.DriveSubsystem;
import ca.warp7.frc2025.subsystems.drive.GyroIO;
import ca.warp7.frc2025.subsystems.drive.GyroIOPigeon2;
import ca.warp7.frc2025.subsystems.drive.ModuleIO;
import ca.warp7.frc2025.subsystems.drive.ModuleIOSim;
import ca.warp7.frc2025.subsystems.drive.ModuleIOTalonFX;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    // Subsystems
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;

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
                        new RollersIOTalonFX(0, ""),
                        new ObjectDectionIOLaserCAN(0, "Top"),
                        new ObjectDectionIOLaserCAN(0, "Front"));
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
                break;
            default:
                drive = null;
                intake = null;
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

        // controller.leftTrigger().and(intake.topSensorTrigger().negate()).(intake.runVoltsRoller(6));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
