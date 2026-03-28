package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  private static final Mode simType = Mode.SIM;
  public static final boolean lowCeiling = true; // Keep shots below 9' to avoid ceiling
  public static final boolean tuningMode = true;
  public static final boolean hootLogging = true;

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

  public final class RollerSystem_Slot0Configs {
    public static final Slot0Configs INTAKE =
        new Slot0Configs().withKP(.2).withKI(0).withKD(0).withKS(0).withKV(0.13).withKA(0);
    public static final Slot0Configs SPINDEXER =
        new Slot0Configs().withKP(.2).withKI(0).withKD(0).withKS(0).withKV(0.13).withKA(0);
    public static final Slot0Configs FEEDER =
        new Slot0Configs().withKP(.2).withKI(0).withKD(0).withKS(0).withKV(0.13).withKA(0);
  }
}
