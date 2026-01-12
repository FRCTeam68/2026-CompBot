package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import frc.robot.subsystems.vision.VisionConstants.ObjectObservationType;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final CameraInfo cameraInfo;
  private final DoubleArrayPublisher orientationPublisher;
  private final IntegerPublisher throttlePublisher;
  private final DoublePublisher pipelinePublisher;
  private final IntegerSubscriber pipelineSubscriber;
  private final DoubleArraySubscriber hardwareSubscriber;
  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;
  private final DoubleArraySubscriber objectSubscriber;

  private Supplier<Rotation2d> rotationSupplier = () -> new Rotation2d();

  /**
   * Creates a new Limelight camera.
   *
   * @param cameraInfo Camera information
   */
  public VisionIOLimelight(CameraInfo cameraInfo) {
    this.cameraInfo = cameraInfo;

    var table = NetworkTableInstance.getDefault().getTable(cameraInfo.name);
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    throttlePublisher = table.getIntegerTopic("throttle_set").publish();
    pipelinePublisher = table.getDoubleTopic("pipeline").publish();
    pipelineSubscriber = table.getIntegerTopic("getpipe").subscribe(0);
    hardwareSubscriber =
        table.getDoubleArrayTopic("hw").subscribe(new double[] {0.0, 0.0, 0.0, 0.0});
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    objectSubscriber = table.getDoubleArrayTopic("rawdetections").subscribe(new double[] {});
  }

  @Override
  public void initRotationSupplier(Supplier<Rotation2d> rotationSupplier) {
    this.rotationSupplier = rotationSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update active pipeline
    inputs.pipelineIndex = (int) pipelineSubscriber.get();

    // Update hardware metrics
    inputs.ramUsage = hardwareSubscriber.get()[2];
    inputs.cpuTemperature = hardwareSubscriber.get()[1];
    inputs.Temperature = hardwareSubscriber.get()[3];

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});

    // Increases network traffic but recommended by Limelight
    NetworkTableInstance.getDefault().flush();

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
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
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }

    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
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
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_2));
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

    // Read new object observations from NetworkTables
    List<ObjectObservation> objectObservations = new LinkedList<>();

    var rawSample = objectSubscriber.get(new double[] {});
    for (int i = 0; i < rawSample.length; i += 12) {
      // TODO: make sure this works
      ObjectObservationType objectObservationType;
      try {
        objectObservationType = ObjectObservationType.values()[(int) rawSample[i]];
      } catch (Exception e) {
        objectObservationType = ObjectObservationType.values()[0];
      }

      objectObservations.add(
          new ObjectObservation(
              // Center X
              rawSample[i + 1],

              // Center y
              rawSample[i + 2],

              // Width
              rawSample[i + 6] - rawSample[i + 4],

              // Height
              rawSample[i + 11] - rawSample[i + 5],

              // Observation id
              objectObservationType));
      // ObjectObservationType.values()[(int) i]));
    }

    // Save object observations to inputs object
    inputs.objectObservations = new ObjectObservation[objectObservations.size()];
    for (int i = 0; i < objectObservations.size(); i++) {
      inputs.objectObservations[i] = objectObservations.get(i);
    }
  }

  @Override
  public CameraInfo getCameraInfo() {
    return cameraInfo;
  }

  @Override
  public void setPipline(int pipelineIndex) {
    pipelinePublisher.accept(pipelineIndex);
  }

  @Override
  public void setThrottle(int skippedFrames) {
    throttlePublisher.accept(skippedFrames);
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
