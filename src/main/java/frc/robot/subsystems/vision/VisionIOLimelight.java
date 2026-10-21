package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final CameraInfo cameraInfo;
  private final DoubleArrayPublisher orientationPublisher;
  private final DoubleArraySubscriber hardwareSubscriber;
  private final DoubleArrayPublisher rewindPublisher;
  private final DoubleArraySubscriber rewindSubscriber;
  private final DoubleSubscriber latencySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;
  private final DoubleArraySubscriber cameraPoseMT2Subscriber;

  private Supplier<double[]> gyroOrientationSupplier =
      () -> new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  /**
   * Creates a new Limelight camera.
   *
   * @param cameraInfo Camera information
   */
  public VisionIOLimelight(CameraInfo cameraInfo) {
    this.cameraInfo = cameraInfo;

    var table = NetworkTableInstance.getDefault().getTable(cameraInfo.name);
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    hardwareSubscriber =
        table.getDoubleArrayTopic("hw").subscribe(new double[] {0.0, 0.0, 0.0, 0.0});
    rewindPublisher = table.getDoubleArrayTopic("capture_rewind").publish();
    rewindSubscriber =
        table.getDoubleArrayTopic("capture_rewind").subscribe(new double[] {0.0, 0.0});
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    cameraPoseMT2Subscriber =
        table.getDoubleArrayTopic("camerapose_field_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void initOrientationSupplier(Supplier<double[]> gyroOrientationSupplier) {
    this.gyroOrientationSupplier = gyroOrientationSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update hardware metrics
    inputs.cpuTempCelsius = hardwareSubscriber.get()[0];
    inputs.cpuUsage = hardwareSubscriber.get()[1];
    inputs.ramUsage = hardwareSubscriber.get()[2];
    inputs.fps = hardwareSubscriber.get()[3];

    // Update orientation for MegaTag 2 (yaw°, yawRate°/s, pitch°, pitchRate°/s, roll°, rollRate°/s)
    orientationPublisher.accept(gyroOrientationSupplier.get());

    // Increases network traffic but recommended by Limelight
    NetworkTableInstance.getDefault().flush();

    // Update camera pose in field coordinates (MegaTag2) for extrinsic calibration
    double[] camPoseRaw = cameraPoseMT2Subscriber.get();
    inputs.cameraPoseFieldMT2 = (camPoseRaw.length >= 6) ? parsePose(camPoseRaw) : new Pose3d();

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      int tagCount = (int) rawSample.value[7];
      double avgDist = rawSample.value[9];

      double angleStdDev =
          angularStdDevBaseline.get()
              + avgDist
                  * avgDist
                  * angularStdDevMegatag1Factor.get()
                  * cameraInfo.MTStdDevFactor.get();
      if (tagCount == 0) {
        angleStdDev = Double.POSITIVE_INFINITY;
      } else if (tagCount == 1) {
        // Single-tag MT1 rotation is jittery; penalize it heavily so it doesn't spin your robot
        angleStdDev *= 3;
      } else if (tagCount >= 3) {
        // Multi-tag solvePnP rotation is rock solid with 3 tags; reward it with higher trust
        angleStdDev *= 0.5;
      }

      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, using only the first tag because ambiguity isn't applicable for
              // multitag
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

              // Tag count
              tagCount,

              // Average tag distance
              avgDist,

              // Observation type
              PoseObservationType.MEGATAG_1,

              // Linear std dev (not used for MT1 — rotation is locked to gyro)
              Double.POSITIVE_INFINITY,

              // Angular std dev
              angleStdDev));
    }

    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      int tagCount = (int) rawSample.value[7];
      double avgDist = rawSample.value[9];

      double xyStdDev =
          linearStdDevBaseline.get()
              + avgDist
                  * avgDist
                  * linearStdDevMegatag2Factor.get()
                  * cameraInfo.MTStdDevFactor.get();
      if (tagCount == 0) {
        xyStdDev = Double.POSITIVE_INFINITY;
      } else if (tagCount >= 2) {
        xyStdDev *= 0.8;
      }

      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              tagCount,

              // Average tag distance
              avgDist,

              // Observation type
              PoseObservationType.MEGATAG_2,

              // calculated linear standard deviation
              xyStdDev,

              // Angular std dev (not used for MT2 — rotation comes from gyro)
              Double.POSITIVE_INFINITY));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs object
    inputs.tagIds = new int[tagIds.size()];
    int n = 0;
    for (int id : tagIds) {
      inputs.tagIds[n++] = id;
    }
  }

  @Override
  public CameraInfo getCameraInfo() {
    return cameraInfo;
  }

  @Override
  public void saveRewind() {
    rewindPublisher.accept(new double[] {rewindSubscriber.get()[0] + 1.0, 165});
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
