package frc.robot;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.lights.Lights.Segment;

public final class Constants {
  private static final Mode simType = Mode.SIM;
  public static final boolean lowCeiling = true; // Keep shots low to avoid ceiling
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

  public static final class LEDSegment {
    // standard segments
    public static final Segment LEFT_SIDE = new Segment(8, 44, 1);
    // public static final Segment MIDDLE = new Segment(45, 62, 2).withOverlappingAnimationSlots(4);
    // public static final Segment RIGHT_SIDE =
    //     new Segment(63, 98, 3).withOverlappingAnimationSlots(4);
    public static final Segment ALL =
        new Segment(8, 44, 4); // .withOverlappingAnimationSlots(1, 2, 3);

    // // auton setup
    // public static final Segment AUTON_Y_LEFT =
    //     new Segment(45, 48, 0).withOverlappingAnimationSlots(2, 4);
    // public static final Segment AUTON_R_LEFT =
    //     new Segment(49, 51, 0).withOverlappingAnimationSlots(2, 4);
    // public static final Segment AUTON_X_LEFT =
    //     new Segment(52, 53, 0).withOverlappingAnimationSlots(2, 4);
    // public static final Segment AUTON_X_RIGHT =
    //     new Segment(54, 55, 0).withOverlappingAnimationSlots(2, 4);
    // public static final Segment AUTON_R_RIGHT =
    //     new Segment(56, 58, 0).withOverlappingAnimationSlots(2, 4);
    // public static final Segment AUTON_Y_RIGHT =
    //     new Segment(59, 62, 0).withOverlappingAnimationSlots(2, 4);
  }

  public static final class LEDColor {
    // team colors
    public static final RGBWColor ORANGE = new RGBWColor(255, 142, 36);
    public static final RGBWColor BLUE = new RGBWColor(0, 0, 255);

    // indicator colors
    public static final RGBWColor BLACK = new RGBWColor(0, 0, 0);
    public static final RGBWColor WHITE = new RGBWColor(255, 230, 220);
    public static final RGBWColor GREEN = new RGBWColor(56, 209, 0);
    public static final RGBWColor RED = new RGBWColor(255, 0, 0);
  }
}
