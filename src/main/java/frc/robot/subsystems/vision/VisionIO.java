package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import frc.robot.subsystems.vision.VisionConstants.ObjectObservationType;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public int pipelineIndex = 0;
    public double ramUsage = 0.0;
    public double cpuTemperature = 0.0;
    public double Temperature = 0.0;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    public ObjectObservation[] objectObservations = new ObjectObservation[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2
  }

  /** Represents an object sample. */
  public static record ObjectObservation(
      double txCenterDeg,
      double tyCenterDeg,
      double widthPixels,
      double heightPixels,
      ObjectObservationType type) {}

  /**
   * Initalize robot rotation supplier. This must be called once for every camera to get accurate
   * pose estimation.
   *
   * @param rotationSupplier Robot rotation
   */
  public default void initRotationSupplier(Supplier<Rotation2d> rotationSupplier) {}

  public default void updateInputs(VisionIOInputs inputs) {}

  /**
   * Get general information about the camera.
   *
   * @return Camera information
   */
  public default CameraInfo getCameraInfo() {
    return CameraInfo.LL_2;
  }

  /**
   * Set the vision pipeline the camera will use.
   *
   * @param pipelineIndex Index of the pipeline
   */
  public default void setPipline(int pipelineIndex) {}

  /**
   * Set the number of frames to skip between processed frames. This is used to reduce the tempature
   * of the camera. Outputs are not zeroed during skipped frames.
   *
   * <p><b>Recommend Values:
   *
   * <p>Disabled:</b> 100-200
   *
   * <p><b>Enabled:</b> 0
   *
   * @param skippedFrames Index of the pipeline
   */
  public default void setThrottle(int skippedFrames) {}
}
