package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // Camera information
  // Limelight 4 name must start with "limelight-four" to work with special functions.
  public static enum CameraInfo {
    LL_3G("limelight-threeg", 10.0, 7),
    // ip address: 10.0.68.9
    // Pose (meters / degrees): x:-0.291079 / y:0.273426 / z:0.31586
    //                          roll:0 / pitch:25 / yaw:174

    LL_4("limelight-four", 1, 6);
    // ip address: 10.0.68.8
    // Pose (meters / degrees): x:-0.264950 / y:-0.272987 / z:0.223248
    //                          roll:0 / pitch:20 / yaw:-120

    String name; // Must match name configured on coprocessor
    double MTStdDevFactor; // April tag pose standard deviation multiplier
    int indicatorIndex;

    CameraInfo(String name, double MTStdDevFactor, int indicatorIndex) {
      this.name = name;
      this.MTStdDevFactor = MTStdDevFactor;
      this.indicatorIndex = indicatorIndex;
    }
  }

  // Pose filtering thresholds
  public static final double MT1MinTags = 1;
  public static final double MT1MaxLinearVelocity = 0.25; // Meters per second
  public static final double MT1MaxAngularVelocity =
      Units.degreesToRadians(1); // Radians per second
  public static final double maxZError = 0.25; // Meters

  // Higher standard deviations result in less truested data

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double linearStdDevBaseline = 0.02; // Meters
  public static final double angularStdDevBaseline = 0.06; // Radians

  // Multipliers to apply for MegaTag 1 observations
  public static final double angularStdDevMegatag1Factor = 1.0;

  // Multipliers to apply for MegaTag 2 observations
  public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
}
