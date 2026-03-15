package frc.robot.subsystems.sensors.canrange;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import frc.robot.util.LoggedTunableNumber;

public class CANrangeIOSim implements CANrangeIO {
  private final CANrangeConfiguration config;
  private final LoggedTunableNumber distanceMeters;

  private boolean prevDetected = true;

  public CANrangeIOSim(String name, CANrangeConfiguration config) {
    this.config = config;

    distanceMeters = new LoggedTunableNumber("Simulation/" + name + "CANrange/DistanceMeters", 0.0);
  }

  @Override
  public void updateInputs(CANrangeIOInputs inputs) {
    boolean detected;
    if (distanceMeters.get()
        < config.ProximityParams.ProximityThreshold - config.ProximityParams.ProximityHysteresis) {
      detected = true;
    } else if (distanceMeters.get()
        > config.ProximityParams.ProximityThreshold + config.ProximityParams.ProximityHysteresis) {
      detected = false;
    } else {
      detected = prevDetected;
    }
    prevDetected = detected;

    inputs.connected = true;
    inputs.detected = detected;
    inputs.distanceMeters = distanceMeters.get();
    inputs.signalStrength = 65535;
    inputs.measurementHealth = MeasurementHealthValue.Good;
  }
}
