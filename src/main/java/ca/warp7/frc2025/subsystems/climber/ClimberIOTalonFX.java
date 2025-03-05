package ca.warp7.frc2025.subsystems.climber;

import ca.warp7.frc2025.util.PhoenixUtil;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX pivotMoter;
    private final SparkMax intake;
    private final Servo pivotServo;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final VoltageOut voltageOut =
            new VoltageOut(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
    private final PositionVoltage positionVoltage =
            new PositionVoltage(0).withUpdateFreqHz(50).withEnableFOC(true);

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> pivotVolts;
    private final StatusSignal<Current> pivotCurrent;
    private final StatusSignal<Temperature> temp;

    private final Debouncer debouncer = new Debouncer(0.5);

    public ClimberIOTalonFX(int pivotMoterID, int intakeMoterID, int servoPWM) {
        pivotMoter = new TalonFX(pivotMoterID, "Drivetrain");
        intake = new SparkMax(intakeMoterID, MotorType.kBrushless);
        pivotServo = new Servo(servoPWM);

        position = pivotMoter.getPosition();
        velocity = pivotMoter.getVelocity();
        pivotVolts = pivotMoter.getMotorVoltage();
        pivotCurrent = pivotMoter.getSupplyCurrent();
        temp = pivotMoter.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, pivotVolts, pivotCurrent, temp);

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kG = 0;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;
        config.Slot0.kP = 0;
        config.Slot0.kD = 0;

        config.CurrentLimits.SupplyCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 100.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        PhoenixUtil.tryUntilOk(5, () -> pivotMoter.getConfigurator().apply(config));
        PhoenixUtil.tryUntilOk(5, () -> pivotMoter.setPosition(0));
    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        StatusCode talonCode = BaseStatusSignal.refreshAll(position, velocity, pivotVolts, pivotCurrent, temp);
        inputs.motorConnected = debouncer.calculate(talonCode.isOK());
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
    public void setControlConstants(double kG, double kS, double kV, double kA, double kP, double kD) {
        config.Slot0.kG = kG;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kP = kP;
        config.Slot0.kD = kD;

        PhoenixUtil.tryUntilOk(5, () -> pivotMoter.getConfigurator().apply(config));
    }

    @Override
    public void setPivotPosition(double position) {
        pivotMoter.setControl(positionVoltage.withPosition(position));
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
    public void setServoPosition(double position) {
        pivotServo.setPosition(position);
    }

    @Override
    public void setPivotSpeed(double speed) {
        pivotMoter.set(speed);
    }
}
