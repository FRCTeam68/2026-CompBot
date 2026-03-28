package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.Slot0Configs;
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
  }

  default void updateInputs(RollerSystemIOInputs inputs) {}

  /**
   * Run system at specified voltage.
   *
   * @param volts Voltage to run the motor at.
   */
  default void runVolts(double volts) {}

  default void runVelocity(double velocity) {}

  /** Stop motor with neutral output. */
  default void stop() {}

  /** Set P, I, and D gains for closed loop control on drive motor. */
  public default void setPID(Slot0Configs config0) {}
}
