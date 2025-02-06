package ca.warp7.frc2025.subsystems.Climber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import ca.warp7.frc2025.util.LoggedTunableNumber;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final LoggedTunableNumber P = new LoggedTunableNumber("Climber/P", 0);
    private final LoggedTunableNumber I = new LoggedTunableNumber("Climber/I", 0);
    private final LoggedTunableNumber D = new LoggedTunableNumber("Climber/D", 0);

    private final ProfiledPIDController pivotController = new ProfiledPIDController(
        P.getAsDouble(), 
        I.getAsDouble(), 
        D.getAsDouble(), 
        new TrapezoidProfile.Constraints(0, 0));

    private final ArmFeedforward feedforwardController = new ArmFeedforward(0, 0, 0); 

    
    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    private double goal = 0.0; 

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        double feedForwardVolts = feedforwardController.calculate(inputs.pivotPosition, goal); 
        double feedBackVolts = pivotController.calculate(inputs.pivotVelocityMetersPerSec); 



        io.setPivotVoltage(feedBackVolts + feedForwardVolts);
    }

    public Command runVoltageCommand(double volts) {
        return this.runOnce(() -> io.setIntakeVoltage(volts));
    }

    public Command setPivotPosition(double goal) {
        return this.runOnce(() -> this.goal = goal);
    }
}
