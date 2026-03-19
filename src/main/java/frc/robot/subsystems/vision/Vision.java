package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.FieldConstants;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.Color;
import frc.robot.subsystems.lights.Lights.LEDSegment;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.ElasticUtil;
import frc.robot.util.ElasticUtil.Notification;
import frc.robot.util.ElasticUtil.NotificationLevel;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Supplier<ChassisSpeeds> chassisSpeedSupplier;
  private final Supplier<Boolean> gyroConnectedSupplier;
  private final Lights lights;
  private final VisionIO[] io;
  private final CameraInfo[] cameraInfo;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Debouncer[] connectedDebouncers;
  private final Alert[] disconnectedAlerts;
  private final LEDSegment[] ledSegments;

  @AutoLogOutput(key = "Vision/EnableMT1")
  public boolean enableMT1 = Constants.tuningMode;

  public Vision(
      VisionConsumer consumer,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedSupplier,
      Supplier<Boolean> gyroConnectedSupplier,
      Lights lights,
      VisionIO... io) {
    this.consumer = consumer;
    this.chassisSpeedSupplier = chassisSpeedSupplier;
    this.gyroConnectedSupplier = gyroConnectedSupplier;
    this.lights = lights;
    this.io = io;

    // Initialize camera specific information
    cameraInfo = new CameraInfo[io.length];
    inputs = new VisionIOInputsAutoLogged[io.length];
    connectedDebouncers = new Debouncer[io.length];
    disconnectedAlerts = new Alert[io.length];
    ledSegments = new LEDSegment[io.length];
    for (int i = 0; i < io.length; i++) {
      this.io[i].initRotationSupplier(() -> poseSupplier.get().getRotation());
      cameraInfo[i] = io[i].getCameraInfo();
      inputs[i] = new VisionIOInputsAutoLogged();
      connectedDebouncers[i] = new Debouncer(0.5, DebounceType.kRising);
      disconnectedAlerts[i] =
          new Alert("Camera" + cameraInfo[i].name + " is disconnected.", AlertType.kError);
      ledSegments[i] =
          new LEDSegment(cameraInfo[i].indicatorIndex, cameraInfo[i].indicatorIndex, 0);
    }

    // Initialize save rewind dashboard button
    SmartDashboard.putData(
        "Vision/SaveRewind", Commands.runOnce(() -> saveLimelightRewind()).ignoringDisable(true));
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

    if (DriverStation.isDisabled() || Constants.tuningMode) {
      for (int i = 0; i < io.length; i++) {
        if (!inputs[i].connected) {
          lights.setSolidColor(Color.Dim.RED, ledSegments[i]);
        } else if (inputs[i].poseObservations.length > 0) {
          lights.setSolidColor(Color.Dim.BLUE, ledSegments[i]);
        } else {
          lights.setSolidColor(Color.Dim.GREEN, ledSegments[i]);
        }
      }
    }

    if (!gyroConnectedSupplier.get() && !enableMT1) {
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
          !connectedDebouncers[cameraIndex].calculate(inputs[cameraIndex].connected));

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoseMT1 = new LinkedList<>();
      List<Pose3d> robotPoseMT2 = new LinkedList<>();
      List<Pose3d> robotPoseAcceptedMT1 = new LinkedList<>();
      List<Pose3d> robotPoseAcceptedMT2 = new LinkedList<>();
      List<Pose3d> robotPoseRejectedMT1 = new LinkedList<>();
      List<Pose3d> robotPoseRejectedMT2 = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = FieldConstants.defaultAprilTagType.getLayout().getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/TagPoses", tagPoses.toArray(new Pose3d[0]));

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Reject and do not log pose if 0 tags are seen
        if (observation.tagCount() == 0) {
          continue;
        }

        if (observation.type() == PoseObservationType.MEGATAG_1 && enableMT1) {
          boolean rejectPose =
              // Must see a minimum number of tags
              observation.tagCount() < MT1MinTags
                  // Must not be moving too fast
                  || Math.hypot(
                          chassisSpeedSupplier.get().vxMetersPerSecond,
                          chassisSpeedSupplier.get().vyMetersPerSecond)
                      > MT1MaxLinearVelocity
                  || chassisSpeedSupplier.get().omegaRadiansPerSecond > MT1MaxAngularVelocity
                  // Must have realistic Z coordinate
                  || Math.abs(observation.pose().getZ()) > maxZError
                  // Must be within the field boundaries
                  || observation.pose().getX() < 0.0
                  || observation.pose().getX() > FieldConstants.fieldLength
                  || observation.pose().getY() < 0.0
                  || observation.pose().getY() > FieldConstants.fieldWidth;

          robotPoseMT1.add(observation.pose());
          if (!rejectPose) {
            robotPoseAcceptedMT1.add(observation.pose());
          } else {
            robotPoseRejectedMT1.add(observation.pose());
          }

          // Only apply measurement if pose is not rejected
          if (!rejectPose) {
            // Calculate standard deviations
            final double linearStdDev = Double.POSITIVE_INFINITY;
            final double angularStdDev =
                angularStdDevBaseline
                    * (Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount())
                    * angularStdDevMegatag1Factor
                    * cameraInfo[cameraIndex].MTStdDevFactor;

            // Send vision observation
            consumer.accept(
                observation.pose().toPose2d(),
                observation.timestamp(),
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
          }

        } else if (observation.type() == PoseObservationType.MEGATAG_2) {
          boolean rejectPose =
              // Must have realistic Z coordinate
              Math.abs(observation.pose().getZ()) > maxZError
                  // Must be within the field boundaries
                  || observation.pose().getX() < 0.0
                  || observation.pose().getX() > FieldConstants.fieldLength
                  || observation.pose().getY() < 0.0
                  || observation.pose().getY() > FieldConstants.fieldWidth;

          robotPoseMT2.add(observation.pose());
          if (!rejectPose) {
            robotPoseAcceptedMT2.add(observation.pose());
          } else {
            robotPoseRejectedMT2.add(observation.pose());
          }

          // Only apply measurement if pose is not rejected
          if (!rejectPose) {
            // Calculate standard deviations
            final double linearStdDev =
                linearStdDevBaseline
                    * (Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount())
                    * linearStdDevMegatag2Factor
                    * cameraInfo[cameraIndex].MTStdDevFactor;
            final double angularStdDev = Double.POSITIVE_INFINITY;

            // Send vision observation
            consumer.accept(
                observation.pose().toPose2d(),
                observation.timestamp(),
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
          }
        }
      }

      if (enableMT1) {
        Logger.recordOutput(
            "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag1/RobotPose",
            robotPoseMT1.toArray(new Pose3d[0]));
        Logger.recordOutput(
            "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag1/RobotPoseAccepted",
            robotPoseAcceptedMT1.toArray(new Pose3d[0]));
        Logger.recordOutput(
            "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag1/RobotPoseRejected",
            robotPoseRejectedMT1.toArray(new Pose3d[0]));
      }

      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag2/RobotPose",
          robotPoseMT2.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag2/RobotPoseAccepted",
          robotPoseAcceptedMT2.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag2/RobotPoseRejected",
          robotPoseRejectedMT2.toArray(new Pose3d[0]));

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoseMT1);
      allRobotPoses.addAll(robotPoseMT2);
      allRobotPosesAccepted.addAll(robotPoseAcceptedMT1);
      allRobotPosesAccepted.addAll(robotPoseAcceptedMT2);
      allRobotPosesRejected.addAll(robotPoseRejectedMT1);
      allRobotPosesRejected.addAll(robotPoseRejectedMT2);
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
