package ca.warp7.frc2025.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX pivotMoter;
    private final SparkMax intake;

    private final VoltageOut voltageOut =
            new VoltageOut(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
    private final MotionMagicVoltage positionVoltageOut =
            new MotionMagicVoltage(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> pivotVolts;
    private final StatusSignal<Current> pivotCurrent;
    private final StatusSignal<Temperature> temp;

    private final Debouncer debouncer = new Debouncer(0.5);

    public ClimberIOTalonFX(int pivotMoterID, int intakeMoterID) {
        pivotMoter = new TalonFX(pivotMoterID, "Drivetrain");
        intake = new SparkMax(intakeMoterID, MotorType.kBrushless);

        position = pivotMoter.getPosition();
        velocity = pivotMoter.getVelocity();
        pivotVolts = pivotMoter.getMotorVoltage();
        pivotCurrent = pivotMoter.getSupplyCurrent();
        temp = pivotMoter.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, pivotVolts, pivotCurrent, temp);
    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        StatusCode talonCode = BaseStatusSignal.refreshAll(position, velocity, pivotVolts, pivotCurrent, temp);
        inputs.pivotRotation = position.getValueAsDouble();
        inputs.pivotVelocityRotationsPerSecond = velocity.getValueAsDouble();
        inputs.pivotVoltage = pivotVolts.getValueAsDouble();
        inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();
        inputs.pivotTempC = temp.getValueAsDouble();

        inputs.intakeVoltage = intake.getBusVoltage();
        inputs.intakeAmps = intake.getOutputCurrent();
        inputs.intakeTempC = intake.getMotorTemperature();
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotMoter.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intake.setVoltage(volts);
    }

    @Override
    public void setPivotPosition(double position) {
        pivotMoter.setControl(positionVoltageOut.withPosition(position));
    }
}
