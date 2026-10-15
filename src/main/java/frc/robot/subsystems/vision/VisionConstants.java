package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class VisionConstants {
  // Camera information
  // Limelight 4 name must start with "limelight-four" to work with special functions.
  public static enum CameraInfo {
    LL_3G("limelight-threeg", 1.0, 7),
    // ip address: 10.0.68.9
    // Pose (meters / degrees): x:-0.291079 / y:0.273426 / z:0.31586
    //                          roll:0 / pitch:25 / yaw:174

    LL_4("limelight-four", 1.0, 6);
    // ip address: 10.0.68.8
    // Pose (meters / degrees): x:-0.264950 / y:-0.272987 / z:0.223248
    //                          roll:0 / pitch:20 / yaw:-120

    String name; // Must match name configured on coprocessor
    LoggedTunableNumber MTStdDevFactor; // April tag pose standard deviation multiplier
    int indicatorIndex;

    CameraInfo(String name, double mtStdDevFactorDefault, int indicatorIndex) {
      this.name = name;
      this.MTStdDevFactor =
          new LoggedTunableNumber("Vision/" + name + "/MTStdDevFactor", mtStdDevFactorDefault);
      this.indicatorIndex = indicatorIndex;
    }
  }

  // Pose filtering thresholds
  public static final double MT1MinTags = 2;
  public static final double MT1MaxLinearVelocity = 0.25; // Meters per second
  public static final double MT1MaxAngularVelocity =
      Units.degreesToRadians(1); // Radians per second
  public static final double maxZError = 0.25; // Meters

  // Higher standard deviations result in less truested data

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final LoggedTunableNumber linearStdDevBaseline =
      new LoggedTunableNumber("Vision/LinearStdDevBaseline", 0.3); // 0.02
  public static final LoggedTunableNumber angularStdDevBaseline =
      new LoggedTunableNumber("Vision/AngularStdDevBaseline", 0.1); // 0.06

  // Multipliers to apply for MegaTag 1 observations
  public static final LoggedTunableNumber angularStdDevMegatag1Factor =
      new LoggedTunableNumber("Vision/AngularStdDevMegatag1Factor", 0.4);

  // Multipliers to apply for MegaTag 2 observations
  public static final LoggedTunableNumber linearStdDevMegatag2Factor =
      new LoggedTunableNumber("Vision/LinearStdDevMegatag2Factor", 0.25);
}
