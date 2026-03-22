package frc.robot.subsystems.sensors.canrange;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.PhoenixUtil;

public class CANrangeIOReal implements CANrangeIO {
  private final CANrange canrange;

  private final StatusSignal<Boolean> detected;
  private final StatusSignal<Distance> distanceMeters;
  private final StatusSignal<Double> ambientSignal;
  private final StatusSignal<Distance> stdDevMeters;
  private final StatusSignal<Double> signalStrength;
  private final StatusSignal<MeasurementHealthValue> measurementHealth;

  public CANrangeIOReal(int id, CANBus canBus, CANrangeConfiguration config) {
    canrange = new CANrange(id, canBus);

    tryUntilOk(5, () -> canrange.getConfigurator().apply(config));

    detected = canrange.getIsDetected();
    distanceMeters = canrange.getDistance();
    ambientSignal = canrange.getAmbientSignal();
    stdDevMeters = canrange.getDistanceStdDev();
    signalStrength = canrange.getSignalStrength();
    measurementHealth = canrange.getMeasurementHealth();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                detected,
                distanceMeters,
                ambientSignal,
                stdDevMeters,
                signalStrength,
                measurementHealth));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(canrange));
    PhoenixUtil.registerSignals(
        canBus,
        detected,
        distanceMeters,
        ambientSignal,
        stdDevMeters,
        signalStrength,
        measurementHealth);
  }

  @Override
  public void updateInputs(CANrangeIOInputs inputs) {
    inputs.connected = BaseStatusSignal.isAllGood(detected, distanceMeters);
    inputs.detected = detected.getValue();
    inputs.distanceMeters = distanceMeters.getValueAsDouble();
    inputs.ambientSignal = ambientSignal.getValueAsDouble();
    inputs.stdDevMeters = stdDevMeters.getValueAsDouble();
    inputs.signalStrength = signalStrength.getValueAsDouble();
    inputs.measurementHealth = measurementHealth.getValue();
  }
}
