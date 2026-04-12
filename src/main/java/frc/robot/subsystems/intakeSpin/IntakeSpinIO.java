package frc.robot.subsystems.intakeSpin;

import com.ctre.phoenix6.configs.SlotConfigs;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeSpinIO {
  @AutoLog
  static class IntakeSpinIOInputs {
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
    public boolean leaderFaultRotorFault1 = false;
    public boolean leaderFaultRotorFault2 = false;
    public boolean followerFaultRotorFault1 = false;
    public boolean followerFaultRotorFault2 = false;
  }

  default void updateInputs(IntakeSpinIOInputs inputs) {}

  /**
   * Run system at specified voltage.
   *
   * @param volts Voltage to run the motor at.
   */
  default void runVolts(double volts) {}

  /**
   * Run system to specified velocity (rotations per second).
   *
   * @param velocity rotations per second
   * @param slot PID slot/index to use for closed-loop control (if applicable)
   */
  default void runVelocity(double velocity, int slot) {}

  /**
   * Configure PID slots on the motor controller. Accepts zero to three SlotConfigs corresponding to
   * slot0, slot1, slot2.
   */
  default void setPID(SlotConfigs... newConfig) {}

  /** Stop motor with neutral output. */
  default void stop() {}
}
