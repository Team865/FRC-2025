package ca.warp7.frc2025.subsystems.Climber;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import ca.warp7.frc2025.Constants.Climber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimberIOSim implements ClimberIO {
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(2),
            Climber.CLIMBER_GEAR_RATIO,
            0.0,
            Climber.CLIMBER_ARM_LENGTH_METERS,
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(360.0),
            true,
            Units.degreesToRadians(0.0));

    private double climberAppliedVolts;
    private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);
    private final ProfiledPIDController pid =
            new ProfiledPIDController(5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(10.0, 10.0));

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        armSim.update(0.02);
        inputs.velocityRadPerSec = RadiansPerSecond.of(armSim.getAngleRads()).in(RadiansPerSecond);
        inputs.position = Rotation2d.fromRadians(armSim.getAngleRads());
        inputs.climberCurrentAmps = armSim.getCurrentDrawAmps();
        inputs.climberAppliedVolts = climberAppliedVolts;
    }

    @Override
    public void setVoltage(double volts) {
        double climberAppliedVolts = MathUtil.clamp(volts, -12, -12);
        armSim.setInputVoltage(volts);
    }

    @Override
    public void setPosition(final double position) {
        setVoltage(pid.calculate(
                        armSim.getAngleRads(),
                        Math.asin((Units.rotationsToRadians(position) * Climber.CLIMBER_DRUM_RADIUS_METERS)
                                / Climber.CLIMBER_ARM_LENGTH_METERS))
                + feedforward.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity));
    }
}
