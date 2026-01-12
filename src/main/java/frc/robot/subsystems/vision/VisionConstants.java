package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleBinaryOperator;

public class VisionConstants {
  // Camera information
  public static enum CameraInfo {
    // Intake camera pose - new Pose3d(new Translation3d(-0.425914, 0.0, 0.254286), new
    //                      Rotation3d(0.0, Units.degreesToRadians(-15.0), Math.PI))
    LL_2(
        "limelight-two",
        new Pose3d(
            new Translation3d(-0.425914, 0.0, 0.254286),
            new Rotation3d(0.0, Units.degreesToRadians(-35.0), Math.PI)),
        1.0,
        new double[] {}),
    LL_3G(
        "limelight-threeg",
        new Pose3d(
            new Translation3d(-0.425914, 0.0, 0.254286),
            new Rotation3d(0.0, Units.degreesToRadians(-15.0), Math.PI)),
        1.0,
        new double[] {}),
    LL_4(
        "limelight-four",
        new Pose3d(
            new Translation3d(-0.425914, 0.0, 0.254286),
            new Rotation3d(0.0, Units.degreesToRadians(-15.0), Math.PI)),
        1.0,
        new double[] {0, 100, 0, 100});

    String name; // Must match name configured on coprocessor
    Pose3d pose; // Camera pose relative to the robot wheelbase
    double MTStdDevFactor; // April tag pose standard deviation multiplier
    double[] objectDetectionEdges; // bottom, top, left, right

    CameraInfo(String name, Pose3d pose, double MTStdDevFactor, double[] objectDetectionEdges) {
      this.name = name;
      this.pose = pose;
      this.MTStdDevFactor = MTStdDevFactor;
      this.objectDetectionEdges = objectDetectionEdges;
    }
  }

  // Pose filtering thresholds
  public static final double MT1MinTags = 2;
  public static final double MT1MaxLinearVelocity = 0.5; // Meters per second
  public static final double MT1MaxAngularVelocity =
      Units.degreesToRadians(5); // Radians per second
  public static final double MT1MaxAverageTagDistance = 2.5; // Meters
  public static final double maxZError = 0.5; // Meters

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

  // Object Detection
  // List of object tags in the same order of the file uploaded to the limelight
  public static enum ObjectObservationType {
    ALGAE,
    NOTE
  }

  public static final DoubleBinaryOperator distanceEquationNote = (x, y) -> x;
}
