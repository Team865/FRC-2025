package ca.warp7.frc2025.subsystems.elevator;

import ca.warp7.frc2025.util.PhoenixUtil;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
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

    private final Debouncer connectedDebouncer = new Debouncer(0.5);

    public ElevatorIOTalonFX(int leftMotorID, int rightMotorID) {
        talon = new TalonFX(leftMotorID, "rio");
        followerTalon = new TalonFX(rightMotorID, "rio");
        followerTalon.setControl(new Follower(talon.getDeviceID(), true));

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio =
                ElevatorSubsystem.GEAR_RATIO / (2 * Math.PI * ElevatorSubsystem.DRUM_RADIUS_METERS);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        config.CurrentLimits.SupplyCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> followerTalon.getConfigurator().apply(config, 0.25));
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
        ParentDevice.optimizeBusUtilizationForAll(talon, followerTalon);
    }

    @Override
    public void updateInputs(ElevatorIOInputAutoLogged inputs) {
        boolean connected = BaseStatusSignal.refreshAll(
                        position, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp)
                .isOK();
        boolean followerConnected = BaseStatusSignal.refreshAll(
                        followerAppliedVolts, followerTorqueCurrent, followerSupplyCurrent, followerTemp)
                .isOK();

        inputs.motorConnected = connectedDebouncer.calculate(connected);
        inputs.followerConnected = connectedDebouncer.calculate(followerConnected);
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
    public void setVoltage(double volts) {
        talon.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setPID(double P, double I, double D) {
        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    }
}
