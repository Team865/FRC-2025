package ca.warp7.frc2025.subsystems.intake;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class ObjectDectionIOLaserCAN implements ObjectDectionIO {
    private LaserCan laserCan;
    private String name;

    private final Alert laserCanStatus;

    public ObjectDectionIOLaserCAN(int laserCANId, String name) {
        this.name = name + "LaserCAN";
        laserCan = new LaserCan(laserCANId);

        laserCanStatus = new Alert("Unset", AlertType.kError);

        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 8, 8));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            new Alert("Failed to configure " + name + " on intake, measurement may be inconsistent", AlertType.kWarning)
                    .set(true);
        }
    }

    @Override
    public void updateInputs(ObjectDectionIOInputs inputs) {
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        boolean trustMeasurment = true;

        if (measurement != null) {
            switch (measurement.status) {
                case LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT:
                    laserCanStatus.set(false);
                    break;
                case LaserCanInterface.LASERCAN_STATUS_NOISE_ISSUE:
                    laserCanStatus.setText("Noise on " + name + ", may have inaccurent readings");
                    laserCanStatus.set(true);
                    break;
                case LaserCanInterface.LASERCAN_STATUS_WEAK_SIGNAL:
                    laserCanStatus.setText(name + " is receving a week singnal");
                    laserCanStatus.set(true);
                    trustMeasurment = false;
                    break;
                case LaserCanInterface.LASERCAN_STATUS_OUT_OF_BOUNDS:
                    laserCanStatus.setText(name + " detected object outside of bounds");
                    laserCanStatus.set(true);
                    trustMeasurment = false;
                    break;
                case LaserCanInterface.LASERCAN_STATUS_WRAPAROUND:
                    laserCanStatus.setText(name + " detected object that wraped around");
                    laserCanStatus.set(true);
                    trustMeasurment = false;
                    break;
            }
        }

        if (trustMeasurment) {
            inputs.objectDistanceMM = measurement.distance_mm;
        } else {
            inputs.objectDistanceMM = 0;
        }
    }
}
