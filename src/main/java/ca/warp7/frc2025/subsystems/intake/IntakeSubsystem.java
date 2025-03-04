package ca.warp7.frc2025.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private final double frontTopDistanceToCoral = 145;

    public boolean holding = false;

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
    public Trigger topSensorTrigger() {
        return new Trigger(() -> MathUtil.isNear(topDistanceToCoral, topSensorInputs.objectDistanceMM, 50));
    }

    @AutoLogOutput
    public Trigger frontSensorTrigger() {
        return new Trigger(() -> MathUtil.isNear(frontTopDistanceToCoral, frontSensorInputs.objectDistanceMM, 30));
    }

    @AutoLogOutput
    public Trigger holdingTrigger() {
        return new Trigger(() -> holding);
    }

    public Command setHolding(boolean state) {
        return runOnce(() -> holding = state);
    }

    @AutoLogOutput
    public Command runVoltsRoller(double inputVolts) {
        return startEnd(() -> rollersIO.setVolts(inputVolts), () -> rollersIO.setVolts(0));
    }

    @AutoLogOutput
    public Command runTorqueAmpsRoller(double inputAmps) {
        return runOnce(() -> rollersIO.setVolts(inputAmps));
    }
}
