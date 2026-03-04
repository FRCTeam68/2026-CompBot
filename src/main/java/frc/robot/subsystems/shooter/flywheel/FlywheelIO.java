package frc.robot.subsystems.shooter.flywheel;

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
    public double followerSupplyCurrentAmps = 0.0;
    public double leaderTorqueCurrentAmps = 0.0;
    public double followerTorqueCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;
    public double followerTempCelsius = 0.0;
  }

  default void updateInputs(FlyWheelIOInputs inputs) {}

  /**
   * Run system at specified voltage.
   *
   * @param volts Voltage to run the motor at.
   */
  default void runVolts(double volts) {}

  /**
   * Run system to specified velocity.
   *
   * <p><b>Units:</b> Mechanism rotations per second.
   *
   * @param velocity Goal velocity.
   * @param slot PID gain slot to use during motion.
   */
  default void runVelocity(double velocity, int slot) {}

  /** Stop motor with neutral output. */
  default void stop() {}

  /**
   * Set PID slot configs.
   *
   * <p>Gravity type and static feedforward sign are ignored and use static values instead.
   *
   * <p><b>Available slots:</b> [0,2]
   *
   * @param newConfig PID gains
   */
  public default void setPID(SlotConfigs... newConfig) {}
}
