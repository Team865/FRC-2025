package ca.warp7.frc2025.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    private final ClimberIO io; 
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    
    public ClimberSubsystem(ClimberIO io) {
        this.io = io; 
    }

   @Override
   public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
   } 
}
