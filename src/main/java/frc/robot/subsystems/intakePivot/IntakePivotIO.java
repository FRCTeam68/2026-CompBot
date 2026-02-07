package frc.robot.subsystems.intakePivot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  static class IntakePivotIOInputs {
    public boolean connected = false;
    public double positionRots = 0.0;
    public double velocityRotsPerSec = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public MagnetHealthValue magnetHealth = MagnetHealthValue.Magnet_Invalid;
    public double appliedVoltage;
  }

  default void updateInputs(IntakePivotIOInputs inputs) {}

  /**
   * Run motor at volts.
   *
   * @param volts Voltage
   */
  default void runVolts(double volts) {}

  /**
   * Run motor to position.
   *
   * @param position Position in mechanism rotations
   * @param slot
   */
  default void runPosition(double rotations, int slot) {}

  /** Stop motor */
  default void stop() {}

  /**
   * Set the current mechanism position.
   *
   * @param rotations Position in mechanism rotations
   */
  default void setPosition(double rotations) {}

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
