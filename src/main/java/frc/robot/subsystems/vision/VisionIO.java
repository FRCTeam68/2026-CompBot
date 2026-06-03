package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public int pipelineIndex = 0;
    public double cpuTempCelsius = 0.0;
    public double cpuUsage = 0.0;
    public double ramUsage = 0.0;
    public double fps = 0.0;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

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
    MEGATAG_2,
    PHOTONVISION
  }

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
    return CameraInfo.values()[1];
  }

  /**
   * Triggers a rewind to be saved to disc. This is only functional on the Limelight 4.
   *
   * <p><b>Capture Time (sec):</b> 165
   */
  public default void saveRewind() {}
}
