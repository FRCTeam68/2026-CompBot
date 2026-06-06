package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {
  @AutoLog
  static class RollerSystemIOInputs {
    public boolean connected = false;
    public double positionRots = 0.0;
    public double velocityRotsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public boolean faultRotorFault1 = false;
    public boolean faultRotorFault2 = false;
  }

  default void updateInputs(RollerSystemIOInputs inputs) {}

  /**
   * Run system at specified voltage.
   *
   * @param volts Voltage to run the motor at.
   */
  default void runVolts(double volts) {}

  /** Stop motor with neutral output. */
  default void stop() {}

  /** Set neutral mode of the motor (Brake/Coast). */
  default void setBrakeMode(boolean enabled) {}
}
