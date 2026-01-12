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

  /** Run roller at volts */
  default void runVolts(double volts) {}

  /**
   * Run roller at velocity
   *
   * @param velocity Velocity in mechanism rotations per second
   */
  default void runVelocity(double velocity) {}

  /**
   * Run roller to position
   *
   * @param rotations Position in mechanism rotations
   */
  default void runPosition(double rotations) {}

  /** Stop roller */
  default void stop() {}

  /**
   * Set the current mechanism position
   *
   * @param rotations Position in mechanism rotations
   */
  default void setPosition(double rotations) {}

  /** Set slot configs for closed loop control of the roller. */
  default void setPID(Slot0Configs newConfig) {}
}
