package ca.warp7.frc2025.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import ca.warp7.frc2025.util.PhoenixUtil;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RollersIOTalonFX implements RollersIO {
    private final TalonFX talon;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private final Debouncer connectedDebouncer = new Debouncer(0.5);

    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(50.0);
    private final VoltageOut VoltageOut = new VoltageOut(0.0).withUpdateFreqHz(50.0);
    // private final NeutralOut neutralOut = new NeutralOut();

    public RollersIOTalonFX(int talonCANId, String canBus) {
        talon = new TalonFX(talonCANId, canBus);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));

        position = talon.getPosition();
        velocity = talon.getVelocity();
        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();

        PhoenixUtil.tryUntilOk(
                5,
                () -> BaseStatusSignal.setUpdateFrequencyForAll(
                        50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius));

        // PhoenixUtil.tryUntilOk(5, () -> talon.optimizeBusUtilization(0, 1.0));
    }

    @Override
    public void updateInputs(RollersIOInputsAutoLogged inputs) {
        inputs.connected = connectedDebouncer.calculate(BaseStatusSignal.refreshAll(
                        position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
                .isOK());

        inputs.positionRads = position.getValue().in(Radians);
        inputs.velocityRadsPerSec = velocity.getValue().in(RadiansPerSecond);
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    }

    @Override
    public void setTorqueAmps(double amps) {
        talon.setControl(torqueCurrentFOC.withOutput(amps));
    }

    @Override
    public void setVolts(double volts) {
        talon.setControl(VoltageOut.withOutput(volts));
    }
}
