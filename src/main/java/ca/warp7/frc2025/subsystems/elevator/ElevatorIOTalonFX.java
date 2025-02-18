package ca.warp7.frc2025.subsystems.elevator;

import ca.warp7.frc2025.Constants.Elevator;
import ca.warp7.frc2025.util.PhoenixUtil;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX talon;
    private final TalonFX followerTalon;

    // config
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    // Control signals
    private final VoltageOut voltageOut =
            new VoltageOut(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
    private final PositionVoltage positionVoltageOut =
            new PositionVoltage(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);

    // Status Signals
    // type system abuse - these correspond to linear meters, NOT rotations
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> temp;

    private final StatusSignal<Voltage> followerAppliedVolts;
    private final StatusSignal<Current> followerTorqueCurrent;
    private final StatusSignal<Current> followerSupplyCurrent;
    private final StatusSignal<Temperature> followerTemp;

    private final Debouncer connectedDebouncerMain = new Debouncer(0.5);
    private final Debouncer connectedDebouncerFollower = new Debouncer(0.5);

    public ElevatorIOTalonFX(int leftMotorID, int rightMotorID) {
        talon = new TalonFX(leftMotorID, "Drivetrain");
        followerTalon = new TalonFX(rightMotorID, "Drivetrain");
        followerTalon.setControl(new Follower(talon.getDeviceID(), true));

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kG = 0;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;
        config.Slot0.kP = 0;
        config.Slot0.kD = 0;

        config.Feedback.SensorToMechanismRatio = Elevator.GEAR_RATIO / (2 * Math.PI * Elevator.DRUM_RADIUS_METERS);
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        config.CurrentLimits.SupplyCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));
        PhoenixUtil.tryUntilOk(5, () -> followerTalon.getConfigurator().apply(config));
        PhoenixUtil.tryUntilOk(5, () -> talon.setPosition(0));

        position = talon.getPosition();
        velocity = talon.getVelocity();
        appliedVolts = talon.getMotorVoltage();
        torqueCurrent = talon.getTorqueCurrent();
        supplyCurrent = talon.getSupplyCurrent();
        temp = talon.getDeviceTemp();

        followerAppliedVolts = followerTalon.getMotorVoltage();
        followerTorqueCurrent = followerTalon.getTorqueCurrent();
        followerSupplyCurrent = followerTalon.getSupplyCurrent();
        followerTemp = followerTalon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, torqueCurrent, temp);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, followerAppliedVolts, followerTorqueCurrent, followerSupplyCurrent, followerTemp);
        ParentDevice.optimizeBusUtilizationForAll(talon);
    }

    @Override
    public void updateInputs(ElevatorIOInputAutoLogged inputs) {
        StatusCode talonCode =
                BaseStatusSignal.refreshAll(position, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp);

        System.out.println("talon: " + talonCode);

        StatusCode followerCode = BaseStatusSignal.refreshAll(
                followerAppliedVolts, followerTorqueCurrent, followerSupplyCurrent, followerTemp);

        System.out.println("follower: " + followerCode);

        inputs.motorConnected = connectedDebouncerMain.calculate(talonCode.isOK());
        inputs.followerConnected = connectedDebouncerFollower.calculate(followerCode.isOK());
        inputs.positionMeters = position.getValueAsDouble();
        inputs.velocityMetersPerSec = velocity.getValueAsDouble();
        inputs.appliedVolts = new double[] {appliedVolts.getValueAsDouble(), followerAppliedVolts.getValueAsDouble()};
        inputs.torqueCurrentAmps =
                new double[] {torqueCurrent.getValueAsDouble(), followerTorqueCurrent.getValueAsDouble()};
        inputs.supplyCurrentAmps =
                new double[] {supplyCurrent.getValueAsDouble(), followerSupplyCurrent.getValueAsDouble()};
        inputs.tempCelsius = new double[] {temp.getValueAsDouble(), followerTemp.getValueAsDouble()};
    }

    @Override
    public void setPosition(double meters) {
        talon.setControl(positionVoltageOut.withPosition(meters));
    }

    @Override
    public void setVoltage(double volts) {
        talon.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setControlConstants(double kG, double kS, double kV, double kA, double kP, double kD) {
        config.Slot0.kG = kG;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kP = kP;
        config.Slot0.kD = kD;

        PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    }

    @Override
    public void setMotionProfile(double velocity, double acceleration, double jerk) {
        config.MotionMagic.MotionMagicCruiseVelocity = velocity;
        config.MotionMagic.MotionMagicAcceleration = acceleration;
        config.MotionMagic.MotionMagicJerk = jerk;

        PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    }
}
