package frc.robot.subsystems.sensors.canrange;

import com.ctre.phoenix6.signals.MeasurementHealthValue;
import org.littletonrobotics.junction.AutoLog;

public interface CANrangeIO {
  @AutoLog
  static class CANrangeIOInputs {
    public boolean connected = false;
    public boolean detected = false;
    public double distanceMeters = 0.0;
    public double ambientSignal = 0.0;
    public double stdDevMeters = 0.0;
    public double signalStrength = 0.0;
    public MeasurementHealthValue measurementHealth = MeasurementHealthValue.Bad;
  }

  default void updateInputs(CANrangeIOInputs inputs) {}
}
