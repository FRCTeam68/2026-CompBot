package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // Camera information
  public static enum CameraInfo {
    // Limelight 4 name must start with "limelight-four" to work with special functions.
    LL_3G(
        "limelight-threeg",
        new Pose3d(
            new Translation3d(-0.264950, -0.272987, 0.223248),
            new Rotation3d(0.0, Units.degreesToRadians(10.0), Math.PI)),
        1.0),
    LL_4(
        "limelight-four",
        new Pose3d(
            new Translation3d(-0.292719, 0.273426, 0.306539),
            new Rotation3d(0.0, Units.degreesToRadians(30.0), Math.PI)),
        1.0);

    String name; // Must match name configured on coprocessor
    Pose3d pose; // Camera pose relative to the robot wheelbase
    double MTStdDevFactor; // April tag pose standard deviation multiplier

    CameraInfo(String name, Pose3d pose, double MTStdDevFactor) {
      this.name = name;
      this.pose = pose;
      this.MTStdDevFactor = MTStdDevFactor;
    }
  }

  // Pose filtering thresholds
  public static final double MT1MinTags = 2;
  public static final double MT1MaxLinearVelocity = 0.25; // Meters per second
  public static final double MT1MaxAngularVelocity =
      Units.degreesToRadians(1); // Radians per second
  public static final double MT1MaxAverageTagDistance = 2.5; // Meters
  public static final double maxZError = 0.25; // Meters

  // Higher standard deviations result in less truested data

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double linearStdDevBaseline = 0.02; // Meters
  public static final double angularStdDevBaseline = 0.06; // Radians

  // Multipliers to apply for MegaTag 1 observations
  public static final double linearStdDevMegatag1Factor =
      Double.POSITIVE_INFINITY; // Do not use linear data
  public static final double angularStdDevMegatag1Factor = 1.0;

  // Multipliers to apply for MegaTag 2 observations
  public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static final double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
