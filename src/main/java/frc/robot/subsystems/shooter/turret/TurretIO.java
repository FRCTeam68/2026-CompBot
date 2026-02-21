package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  static class TurretIOInputs {
    public boolean motorConnected = false;
    public boolean cancoderConnected = false;
    public double positionDeg = 0.0;
    public double velocityRotsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public MagnetHealthValue magnetHealth = MagnetHealthValue.Magnet_Invalid;
    public double absolutePosition = 0.0;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  /**
   * Run system at specified voltage.
   *
   * @param volts Voltage to run the motor at.
   */
  default void runVolts(double volts) {}

  /**
   * Run system to position.
   *
   * <p><b>Units:</b> Mechanism degrees.
   *
   * @param degrees Goal position.
   * @param slot PID gain slot to use during motion.
   */
  default void runPosition(double degrees, int slot) {}

  /** Stop motor with neutral output. */
  default void stop() {}

  /**
   * Set the current mechanism position.
   *
   * @param degrees Position in mechanism rotations.
   */
  default void setPosition(double degrees) {}

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

  /**
   * Set motion magic configs.
   *
   * @param newConfig Motion magic config
   */
  public default void setMotionMagic(MotionMagicConfigs newConfig) {}
}
