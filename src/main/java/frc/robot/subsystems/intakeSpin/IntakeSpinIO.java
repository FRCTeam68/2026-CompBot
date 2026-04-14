package frc.robot.subsystems.intakeSpin;

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

  /** Stop motor with neutral output. */
  default void stop() {}
}
