package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  static class FlyWheelIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public double positionRots = 0.0;
    public double velocityRotsPerSec = 0.0;
    public double leaderAppliedVoltage = 0.0;
    public double followerAppliedVoltage = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderTorqueCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;
    public double followerTempCelsius = 0.0;
  }

  default void updateInputs(FlyWheelIOInputs inputs) {}

  /**
   * Run motor at volts.
   *
   * @param volts Voltage
   */
  default void runVolts(double volts) {}

  /**
   * Run motor at velocity.
   *
   * @param velocity Velocity in mechanism rotations per second
   * @param slot
   */
  default void runVelocity(double velocity, int slot) {}

  /** Stop motor */
  default void stop() {}

  /**
   * Set PID slot configs.
   *
   * <p>Gravity type and static feedforward sign are ignored and use static values instead.
   *
   * <ul>
   *   <li><b>Available slots:</b> [0,2]
   * </ul>
   *
   * @param newConfig PID gains
   */
  public default void setPID(SlotConfigs... newConfig) {}

  /** Set motion magic velocity, acceleration and jerk. */
  public default void setMotionMagic(MotionMagicConfigs newConfig) {}
}
