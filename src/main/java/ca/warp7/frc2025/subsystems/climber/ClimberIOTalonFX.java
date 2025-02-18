package ca.warp7.frc2025.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX pivotMotor;
    private final TalonFX followerMotor;

    private final StatusSignal<Angle> pivotRotations;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotAmps;
    private final StatusSignal<Temperature> pivotTempCelcius;

    private final StatusSignal<Voltage> followerVoltage;
    private final StatusSignal<Current> followerAmps;
    private final StatusSignal<Temperature> followerTempCelcius;

    private final VoltageOut pivotVoltageOut = new VoltageOut(0.0).withEnableFOC(true);

    private final MotionMagicVoltage pivotMotionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);

    private final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    private final Debouncer connectedDebouncer = new Debouncer(0.5);

    public ClimberIOTalonFX(int pivotMotorID, int followerMotorID) {
        pivotMotor = new TalonFX(pivotMotorID, "Drivetrain");
        followerMotor = new TalonFX(followerMotorID, "Drivetrain");

        pivotRotations = pivotMotor.getPosition();
        pivotVelocity = pivotMotor.getVelocity();

        pivotVoltage = pivotMotor.getMotorVoltage();
        pivotAmps = pivotMotor.getSupplyCurrent();
        pivotTempCelcius = pivotMotor.getDeviceTemp();

        followerVoltage = followerMotor.getMotorVoltage();
        followerAmps = followerMotor.getSupplyCurrent();
        followerTempCelcius = followerMotor.getDeviceTemp();

        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.Slot0.kG = 0.0;
        pivotConfig.Slot0.kV = 0.0;
        pivotConfig.Slot0.kA = 0.0;
        pivotConfig.Slot0.kS = 0.0;
        pivotConfig.Slot0.kP = 0.0;
        pivotConfig.Slot0.kD = 0.0;

        pivotConfig.MotionMagic.MotionMagicAcceleration = 1.0;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;

        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setPosition(ClimberSubsystem.PIVOT_MIN_ANGLE.getRotations());
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                pivotVelocity,
                pivotVoltage,
                pivotAmps,
                pivotTempCelcius,
                followerVoltage,
                followerAmps,
                followerTempCelcius);
    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        boolean connected = BaseStatusSignal.refreshAll(pivotVelocity, pivotVoltage, pivotAmps, pivotTempCelcius)
                .isOK();
        boolean followerConnected = BaseStatusSignal.refreshAll(followerVoltage, followerAmps, followerTempCelcius)
                .isOK();

        inputs.motorConnected = connectedDebouncer.calculate(connected);
        inputs.followerConnected = connectedDebouncer.calculate(followerConnected);

        inputs.pivotRotation = Rotation2d.fromRotations(pivotRotations.getValueAsDouble());
        inputs.pivotVelocityRotationsPerSecond = pivotVelocity.getValueAsDouble();

        inputs.pivotVoltage = new double[] {pivotVoltage.getValueAsDouble(), followerVoltage.getValueAsDouble()};
        inputs.pivotCurrentAmps = new double[] {pivotAmps.getValueAsDouble(), followerAmps.getValueAsDouble()};
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotMotor.setControl(pivotVoltageOut.withOutput(volts));
    }

    @Override
    public void setPivotSetpoint(Rotation2d rotation) {
        pivotMotor.setControl(pivotMotionMagic.withPosition(rotation.getRotations()));
    }
}
