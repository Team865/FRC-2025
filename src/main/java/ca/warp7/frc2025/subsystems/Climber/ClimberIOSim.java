package ca.warp7.frc2025.subsystems.Climber;

import ca.warp7.frc2025.Constants.Climber;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimberIOSim implements ClimberIO {
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(2),
            Climber.CLIMBER_PIVOT_GEAR_RATIO,
            0.1,
            Climber.CLIMBER_ARM_LENGTH_METERS,
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(360.0),
            true,
            Units.degreesToRadians(0.0));

    DCMotorSim climbIntakeSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.01, Climber.CLIMBER_INTAKE_GEAR_RATIO),
        DCMotor.getNeo550(1),
        Climber.CLIMBER_INTAKE_GEAR_RATIO);
    
    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    }
}
