package ca.warp7.frc2025.subsystems.Climber;

import ca.warp7.frc2025.Constants.Climber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimberIOSim implements ClimberIO {

    private double pivotAppliedVolts = 0.0;

    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(2),
            Climber.CLIMBER_PIVOT_GEAR_RATIO,
            0.1,
            Climber.CLIMBER_ARM_LENGTH_METERS,
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(360.0),
            true,
            Units.degreesToRadians(0.0));

    DCMotorSim intakeSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.01, Climber.CLIMBER_INTAKE_GEAR_RATIO),
            DCMotor.getNeo550(1),
            Climber.CLIMBER_INTAKE_GEAR_RATIO,
            0.0);

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        pivotSim.update(0.20);
        intakeSim.update(0.20);

        inputs.pivotAppliedVolts = new double[] {pivotAppliedVolts};
        inputs.pivotCurrentAmps = new double[] {pivotSim.getCurrentDrawAmps()};

        inputs.climbIntakeVolts = intakeSim.getInputVoltage();
        inputs.climbIntakeCurrentAmps = intakeSim.getCurrentDrawAmps();
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotAppliedVolts = MathUtil.clamp(volts, -12, 12);
        pivotSim.setInputVoltage(pivotAppliedVolts);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeSim.setInputVoltage(volts);
    }
}
