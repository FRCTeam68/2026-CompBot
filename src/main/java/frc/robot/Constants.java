package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.FieldConstants.FieldType;

public final class Constants {
  private static final Mode simType = Mode.SIM;
  public static final FieldType fieldtype = FieldType.WELDED;
  public static final Boolean tuningMode = true;
  public static final Double loopPeriodSecs = 0.02;
  public static final Double warningTempCelsius = 60.0;

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

  public static final class RobotSize {
    public static final double width =
        Units.inchesToMeters(27 + 5 + 5); // 27" + 5" bumpers both sides
    public static final double length = Units.inchesToMeters(27 + 5 + 5);
  }
}
