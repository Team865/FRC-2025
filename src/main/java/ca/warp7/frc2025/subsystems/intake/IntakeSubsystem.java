package ca.warp7.frc2025.subsystems.intake;

import ca.warp7.frc2025.Robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    // Roller io
    private final RollersIO rollersIO;
    private final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

    // Sensors
    private final ObjectDectionIO topSensorIO;
    private final ObjectDectionIO frontSensorIO;

    private final ObjectDectionIOInputsAutoLogged topSensorInputs = new ObjectDectionIOInputsAutoLogged();
    private final ObjectDectionIOInputsAutoLogged frontSensorInputs = new ObjectDectionIOInputsAutoLogged();

    private final Alert disconnectedMotor;

    private final double topDistanceToCoral = 95;
    private final double frontTopDistanceToCoral = 50;

    public IntakeSubsystem(RollersIO rollersIO, ObjectDectionIO topSensorIO, ObjectDectionIO frontSensorIO) {
        this.rollersIO = rollersIO;
        this.topSensorIO = topSensorIO;
        this.frontSensorIO = frontSensorIO;

        disconnectedMotor = new Alert("Intake motor disconnected", AlertType.kError);
    }

    @Override
    public void periodic() {
        rollersIO.updateInputs(rollersInputs);
        topSensorIO.updateInputs(topSensorInputs);
        frontSensorIO.updateInputs(frontSensorInputs);

        Logger.processInputs("Intake/Rollers", rollersInputs);
        Logger.processInputs("Intake/TopSensor", topSensorInputs);
        Logger.processInputs("Intake/FrontSensor", frontSensorInputs);

        disconnectedMotor.set(!rollersInputs.connected);
    }

    @AutoLogOutput
    public Trigger bottomSensorTrigger() {
        return new Trigger(() -> MathUtil.isNear(topDistanceToCoral, topSensorInputs.objectDistanceMM, 50));
    }

    @AutoLogOutput
    public Trigger middleSensorTrigger() {
        return new Trigger(() -> MathUtil.isNear(frontTopDistanceToCoral, frontSensorInputs.objectDistanceMM, 30));
    }

    @AutoLogOutput
    public Command runVoltsRoller(double inputVolts) {
        return startEnd(() -> rollersIO.setVolts(inputVolts), () -> rollersIO.setVolts(0));
    }

    public Command setVoltsRoller(double volts) {
        return runOnce(() -> rollersIO.setVolts(volts));
    }

    @AutoLogOutput
    public Command runTorqueAmpsRoller(double inputAmps) {
        return runOnce(() -> rollersIO.setVolts(inputAmps));
    }

    private void setBottomSensor(boolean value) {
        if (Robot.isSimulation() && value) {
            topSensorInputs.objectDistanceMM = topDistanceToCoral;
        } else {
            topSensorInputs.objectDistanceMM = 0;
        }
    }

    private void setMiddleSensor(boolean value) {
        if (Robot.isSimulation() && value) {
            frontSensorInputs.objectDistanceMM = frontTopDistanceToCoral;
        } else {
            frontSensorInputs.objectDistanceMM = 0;
        }
    }

    public Command intake() {
        return (Robot.isSimulation()
                        ? new WaitCommand(0.5)
                                .andThen(() -> setMiddleSensor(true))
                                .andThen(() -> setBottomSensor(true))
                                .andThen(new PrintCommand("Simulating intake"))
                        : Commands.none())
                .andThen(runVoltsRoller(-4).until(bottomSensorTrigger()));
    }

    public Command outake() {
        return (Robot.isSimulation()
                        ? new WaitCommand(0.5)
                                .andThen(() -> setMiddleSensor(false))
                                .andThen(() -> setBottomSensor(false))
                                .andThen(new PrintCommand("Simulating outake"))
                        : Commands.none())
                .andThen(runVoltsRoller(-10)
                        .until(bottomSensorTrigger()
                                .negate()
                                .and(middleSensorTrigger())
                                .negate()));
    }
}
