package ca.warp7.frc2025.subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX talon;
    private final TalonFX followerTalon;
    private final SparkMax sparkMax;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final VoltageOut voltageOut =
            new VoltageOut(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> angularVelocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;

    private final StatusSignal<Voltage> followerAppliedVolts;
    private final StatusSignal<Current> followerSupplyCurrent;

    private final Debouncer debouncer = new Debouncer(0.5);

    public ClimberIOTalonFX(int leftMoterID, int rightMoterID, int intakeMoterID) {
        talon = new TalonFX(leftMoterID);
        followerTalon = new TalonFX(rightMoterID);
        sparkMax = new SparkMax(intakeMoterID, MotorType.kBrushless);
        followerTalon.setControl(new Follower(talon.getDeviceID(), true));

        position = talon.getPosition();
        angularVelocity = talon.getVelocity();

        appliedVolts = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();

        followerAppliedVolts = followerTalon.getMotorVoltage();
        followerSupplyCurrent = followerTalon.getSupplyCurrent();
    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        boolean motorConnected = BaseStatusSignal.refreshAll(position, angularVelocity, appliedVolts, supplyCurrent)
                .isOK();
        boolean followerConnected = BaseStatusSignal.refreshAll(followerAppliedVolts, followerSupplyCurrent)
                .isOK();

        inputs.motorConnected = debouncer.calculate(motorConnected);
        inputs.followerConnected = debouncer.calculate(followerConnected);

        inputs.pivotPosition = position.getValueAsDouble();
        inputs.pivotVelocityMetersPerSec = angularVelocity.getValueAsDouble();

        inputs.pivotAppliedVolts =
                new double[] {appliedVolts.getValueAsDouble(), followerAppliedVolts.getValueAsDouble()};
        inputs.pivotCurrentAmps =
                new double[] {supplyCurrent.getValueAsDouble(), followerSupplyCurrent.getValueAsDouble()};
    }

    @Override
    public void setPivotVoltage(double volts) {
        talon.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setIntakeVoltage(double volts) {
        sparkMax.setVoltage(volts);
    }
}
