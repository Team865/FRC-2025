package ca.warp7.frc2025.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim sim = new ElevatorSim(
            4.62, 0.07, DCMotor.getKrakenX60Foc(2), Units.inchesToMeters(0), Units.inchesToMeters(29), true, 0);

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputAutoLogged inputs) {
        sim.update(0.02);

        inputs.positionMeters = sim.getPositionMeters();
        inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12, 12);
        sim.setInput(appliedVolts);
    }

    @Override
    public void setPID(double P, double I, double D) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPID'");
    }
}
