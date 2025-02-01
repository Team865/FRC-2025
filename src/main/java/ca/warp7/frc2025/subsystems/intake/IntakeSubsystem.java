package ca.warp7.frc2025.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

class IntakeSubsytem extends SubsystemBase {
    private final RollersIO rollersIO;
    private final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();
    private final Alert disconnected;

    public IntakeSubsytem(RollersIO rollersIO) {
        this.rollersIO = rollersIO;

        disconnected = new Alert("Intake motor disconnected", AlertType.kError);
    }

    @Override
    public void periodic() {
        rollersIO.updateInputs(rollersInputs);
        Logger.processInputs("Intake", rollersInputs);

        disconnected.set(!rollersInputs.connected);
    }

    @AutoLogOutput
    public Command runVoltsRoller(double inputVolts) {
        return runOnce(() -> rollersIO.setVolts(inputVolts));
    }

    @AutoLogOutput
    public Command runTorqueAmpsRoller(double inputAmps) {
        return runOnce(() -> rollersIO.setVolts(inputAmps));
    }
}
