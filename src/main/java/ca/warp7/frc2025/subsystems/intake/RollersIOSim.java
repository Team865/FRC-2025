package ca.warp7.frc2025.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollersIOSim implements RollersIO {
    private final DCMotorSim sim;
    private final DCMotor gearbox;
    private double appliedVoltage = 0;

    public RollersIOSim(DCMotor motor, double reduction, double moi) {
        gearbox = motor;
        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moi, reduction), motor);
    }

    @Override
    public void updateInputs(RollersIOInputsAutoLogged inputs) {
        sim.update(0.02);

        inputs.connected = true;
        inputs.positionRads = sim.getAngularPositionRad();
        inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
        inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = gearbox.getCurrent(sim.getAngularVelocityRadPerSec(), appliedVoltage);
    }

    @Override
    public void setVolts(double volts) {
        appliedVoltage = MathUtil.clamp(volts, -12, 12);
        sim.setInputVoltage(appliedVoltage);
    }

    // TODO: Change to torque instead of amps
    @Override
    public void setTorqueAmps(double amps) {
        setVolts(gearbox.getVoltage(gearbox.getCurrent(amps), sim.getAngularVelocityRadPerSec()));
    }
}
