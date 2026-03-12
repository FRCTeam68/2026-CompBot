package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.ElasticUtil;
import frc.robot.util.ElasticUtil.Notification;
import frc.robot.util.ElasticUtil.NotificationLevel;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Supplier<ChassisSpeeds> chassisSpeedSupplier;
  private final Supplier<Boolean> gyroConnectedSupplier;
  private final VisionIO[] io;
  private final CameraInfo[] cameraInfo;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Debouncer[] connectedDebouncers;
  private final Alert[] disconnectedAlerts;

  @Getter private Optional<Translation2d> targetNote = Optional.empty();

  @AutoLogOutput(key = "Vision/EnableMT1")
  private boolean enableMT1 = Constants.tuningMode;

  public Vision(
      VisionConsumer consumer,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedSupplier,
      Supplier<Boolean> gyroConnectedSupplier,
      VisionIO... io) {
    this.consumer = consumer;
    this.chassisSpeedSupplier = chassisSpeedSupplier;
    this.gyroConnectedSupplier = gyroConnectedSupplier;
    this.io = io;

    // Initialize camera specific information
    cameraInfo = new CameraInfo[io.length];
    inputs = new VisionIOInputsAutoLogged[io.length];
    connectedDebouncers = new Debouncer[io.length];
    disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      this.io[i].initRotationSupplier(() -> poseSupplier.get().getRotation());
      cameraInfo[i] = io[i].getCameraInfo();
      inputs[i] = new VisionIOInputsAutoLogged();
      connectedDebouncers[i] = new Debouncer(0.5, DebounceType.kRising);
      disconnectedAlerts[i] =
          new Alert("Camera" + cameraInfo[i].name + " is disconnected.", AlertType.kError);
    }

    // Initialize save rewind dashboard button
    SmartDashboard.putData(
        "Vision/SaveRewind", Commands.runOnce(() -> saveLimelightRewind()).ignoringDisable(true));
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public Pose2d getTagPose(int cameraIndex) {
    Pose2d tagPose2d;

    if (inputs[cameraIndex].tagIds.length > 0) {
      int tagId = inputs[cameraIndex].tagIds[0];
      var tagPose = FieldConstants.defaultAprilTagType.getLayout().getTagPose(tagId);

      if (tagPose.isPresent()) {
        tagPose2d = tagPose.get().toPose2d();
      } else {
        tagPose2d = Pose2d.kZero;
      }
    } else {
      tagPose2d = Pose2d.kZero;
    }
    return tagPose2d;
  }

  /**
   * Throttle the number of processed frames. This is used to reduce the tempature of the camera.
   * Outputs are not zeroed during skipped frames. This is only funtional on the Limelight 4.
   */
  public void throttleLimelight(boolean shouldThrottle) {
    for (int i = 0; i < io.length; i++) {
      if (cameraInfo[i].name.startsWith("limelight-four"))
        io[i].setThrottle(shouldThrottle ? 200 : 0);
    }
  }

  /** Save Limelight 4 rewind to disc. This is only functional on the Limelight 4. */
  public void saveLimelightRewind() {
    for (int i = 0; i < io.length; i++) {
      if (cameraInfo[i].name.startsWith("limelight-four")) io[i].saveRewind();
    }
    ElasticUtil.sendNotification(
        new Notification(
            NotificationLevel.INFO, "Rewind Saved", "Saved Limelight 4 rewind to disc."));
  }

  /** Returns true if any camera is connected. */
  public boolean isAnyConnected() {
    for (int i = 0; i < io.length; i++) {
      if (inputs[i].connected) return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(cameraInfo[i].name, inputs[i]);
    }

    if (!gyroConnectedSupplier.get() && DriverStation.isEnabled() && !enableMT1) {
      enableMT1 = true;
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(
          !connectedDebouncers[cameraIndex].calculate(inputs[cameraIndex].connected)
              && Constants.getMode() != Mode.SIM);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPosesMT1 = new LinkedList<>();
      List<Pose3d> robotPosesMT2 = new LinkedList<>();
      List<Pose3d> robotPosesAcceptedMT1 = new LinkedList<>();
      List<Pose3d> robotPosesRejectedMT1 = new LinkedList<>();
      List<Pose3d> robotPosesAcceptedMT2 = new LinkedList<>();
      List<Pose3d> robotPosesRejectedMT2 = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = FieldConstants.defaultAprilTagType.getLayout().getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Reject and do not log pose if 0 tags are seen
        if (observation.tagCount() == 0) {
          continue;
        }

        // Filter out low quality MT1 poses
        boolean rejectMT1Pose =
            observation.type() == PoseObservationType.MEGATAG_1
                && (!enableMT1
                    || observation.tagCount() < MT1MinTags
                    || chassisSpeedSupplier.get().vxMetersPerSecond > MT1MaxLinearVelocity
                    || chassisSpeedSupplier.get().vyMetersPerSecond > MT1MaxLinearVelocity
                    || chassisSpeedSupplier.get().omegaRadiansPerSecond > MT1MaxAngularVelocity);

        boolean rejectPose =
            rejectMT1Pose
                // Must have realistic Z coordinate
                || Math.abs(observation.pose().getZ()) > maxZError
                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > FieldConstants.fieldLength
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > FieldConstants.fieldWidth;

        // Add pose to log
        if (observation.type() == PoseObservationType.MEGATAG_1) {
          robotPosesMT1.add(observation.pose());
          if (rejectPose) {
            robotPosesRejectedMT1.add(observation.pose());
          } else {
            robotPosesAcceptedMT1.add(observation.pose());
          }
        } else if (observation.type() == PoseObservationType.MEGATAG_2) {
          robotPosesMT2.add(observation.pose());
          if (rejectPose) {
            robotPosesRejectedMT2.add(observation.pose());
          } else {
            robotPosesAcceptedMT2.add(observation.pose());
          }
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_1) {
          linearStdDev *= linearStdDevMegatag1Factor;
          angularStdDev *= angularStdDevMegatag1Factor;
        } else if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }

        linearStdDev *= cameraInfo[cameraIndex].MTStdDevFactor;
        angularStdDev *= cameraInfo[cameraIndex].MTStdDevFactor;

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag1/RobotPosesAll",
          robotPosesMT1.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag1/RobotPosesAccepted",
          robotPosesAcceptedMT1.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag1/RobotPosesRejected",
          robotPosesRejectedMT1.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag2/RobotPosesAll",
          robotPosesMT2.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag2/RobotPosesAccepted",
          robotPosesAcceptedMT2.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag2/RobotPosesRejected",
          robotPosesRejectedMT2.toArray(new Pose3d[0]));

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPosesMT1);
      allRobotPoses.addAll(robotPosesMT2);
      allRobotPosesAccepted.addAll(robotPosesAcceptedMT1);
      allRobotPosesAccepted.addAll(robotPosesAcceptedMT2);
      allRobotPosesRejected.addAll(robotPosesRejectedMT1);
      allRobotPosesRejected.addAll(robotPosesRejectedMT2);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPosesAll", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
