package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  private static final Mode simType = Mode.SIM;
  public static final boolean lowCeiling = true; // Keep shots low to avoid ceiling
  public static final boolean tuningMode = true;
  public static final boolean hootLogging = false;

  public static final double loopPeriodSecs = 0.02;
  public static final double loopOverrunWarningSecs = 0.2;

  public static final double warningTempCelsius = 60.0;

  public static Mode getMode() {
    if (RobotBase.isReal()) {
      return Mode.REAL;
    } else {
      return simType;
    }
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a simulated robot. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
