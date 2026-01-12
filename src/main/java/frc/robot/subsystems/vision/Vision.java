package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.LoggedTunableNumber;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedSupplier;
  private final VisionIO[] io;
  private final CameraInfo[] cameraInfo;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Debouncer[] connectedDebouncers;
  private final Alert[] disconnectedAlerts;

  @Getter private Optional<Translation2d> targetNote = Optional.empty();

  // TODO: this currently isn't implemented. We need to see if it is needed.
  @SuppressWarnings("unused")
  private LoggedTunableNumber targetRetainSeconds =
      new LoggedTunableNumber("Vision/TargetRetainSeconds", 0.5);

  public Vision(
      VisionConsumer consumer,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedSupplier,
      VisionIO... io) {
    this.consumer = consumer;
    this.poseSupplier = poseSupplier;
    this.chassisSpeedSupplier = chassisSpeedSupplier;
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
      connectedDebouncers[i] = new Debouncer(0.5, DebounceType.kFalling);
      disconnectedAlerts[i] =
          new Alert("Camera" + cameraInfo[i].name + " is disconnected.", AlertType.kError);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return poseSupplier.get().getRotation().minus(inputs[cameraIndex].latestTargetObservation.tx());
  }

  public Pose2d getTagPose(int cameraIndex) {
    Pose2d tagPose2d;

    if (inputs[cameraIndex].tagIds.length > 0) {
      int tagId = inputs[cameraIndex].tagIds[0];
      var tagPose = FieldConstants.defaultAprilTagLayout.getTagPose(tagId);

      if (tagPose.isPresent()) {
        tagPose2d = tagPose.get().toPose2d();
      } else {
        tagPose2d = new Pose2d();
      }
    } else {
      tagPose2d = new Pose2d();
    }
    return tagPose2d;
  }

  /**
   * Throttle the number of processed frames. This is used to reduce the tempature of the camera.
   * Outputs are not zeroed during skipped frames.
   *
   * <p>This is only applied to the Limelight 4.
   */
  public void setThrottle(boolean throttleCamera) {
    for (int i = 0; i < io.length; i++) {
      if (cameraInfo[i].name == "limelight-four") io[i].setThrottle(throttleCamera ? 200 : 0);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(cameraInfo[i].name, inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    List<Pose3d> allObjectPosesNote = new LinkedList<>();

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
        var tagPose = FieldConstants.defaultAprilTagLayout.getTagPose(tagId);
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
                && (observation.tagCount() < MT1MinTags
                    || observation.averageTagDistance() > MT1MaxAverageTagDistance
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

      // Log camera datadata
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag1/RobotPosesAll",
          robotPosesMT1.toArray(new Pose3d[robotPosesMT1.size()]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag2/RobotPosesAll",
          robotPosesMT2.toArray(new Pose3d[robotPosesMT2.size()]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag1/RobotPosesAccepted",
          robotPosesAcceptedMT1.toArray(new Pose3d[robotPosesAcceptedMT1.size()]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag1/RobotPosesRejected",
          robotPosesRejectedMT1.toArray(new Pose3d[robotPosesRejectedMT1.size()]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag2/RobotPosesAccepted",
          robotPosesAcceptedMT2.toArray(new Pose3d[robotPosesAcceptedMT2.size()]));
      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/MegaTag2/RobotPosesRejected",
          robotPosesRejectedMT2.toArray(new Pose3d[robotPosesRejectedMT2.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPosesMT1);
      allRobotPoses.addAll(robotPosesMT2);
      allRobotPosesAccepted.addAll(robotPosesAcceptedMT1);
      allRobotPosesAccepted.addAll(robotPosesAcceptedMT2);
      allRobotPosesRejected.addAll(robotPosesRejectedMT1);
      allRobotPosesRejected.addAll(robotPosesRejectedMT2);

      List<Pose3d> objectPosesNote = new LinkedList<>();

      for (var observation : inputs[cameraIndex].objectObservations) {
        objectPosesNote.add(
            new Pose3d(poseSupplier.get())
                .transformBy(
                    new Transform3d(
                        new Translation3d(),
                        new Rotation3d(
                            0.0,
                            0.0,
                            cameraInfo[cameraIndex].pose.getRotation().getZ()
                                - Math.atan(
                                    Math.tan(
                                        Units.degreesToRadians(observation.txCenterDeg())
                                            / Math.cos(
                                                cameraInfo[cameraIndex]
                                                    .pose
                                                    .getRotation()
                                                    .getY()))))))
                // Object specific distance and height
                .transformBy(
                    switch (observation.type()) {
                      case ALGAE -> new Transform3d();

                      case NOTE ->
                          new Transform3d(
                              new Translation3d(
                                  VisionConstants.distanceEquationNote.applyAsDouble(
                                      observation.widthPixels(), observation.heightPixels()),
                                  0.0,
                                  Units.inchesToMeters(1)),
                              new Rotation3d());
                    }));
      }

      Logger.recordOutput(
          "Vision/" + cameraInfo[cameraIndex].name + "/ObjectDetection/NotePoses",
          objectPosesNote.toArray(new Pose3d[objectPosesNote.size()]));
      allObjectPosesNote.addAll(objectPosesNote);
    }

    // calculate target object
    List<Pose2d> allObjectPosesNotePose2d = new LinkedList<>();
    for (Pose3d objectPose3d : allObjectPosesNote) {
      allObjectPosesNotePose2d.add(objectPose3d.toPose2d());
    }

    targetNote =
        switch (allObjectPosesNote.size()) {
          case 0 -> Optional.empty();

          case 1 -> Optional.of(allObjectPosesNotePose2d.get(0).getTranslation());

          default ->
              Optional.of(poseSupplier.get().nearest(allObjectPosesNotePose2d).getTranslation());
        };

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAll", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    Logger.recordOutput(
        "Vision/Summary/NotePosesAll",
        allObjectPosesNote.toArray(new Pose3d[allObjectPosesNote.size()]));
    Logger.recordOutput(
        "Vision/Summary/NoteTarget",
        targetNote.isPresent()
            ? new Pose3d[] {
              new Pose3d(new Pose2d(targetNote.get(), new Rotation2d()))
                  .transformBy(new Transform3d(0.0, 0.0, Units.inchesToMeters(1), new Rotation3d()))
            }
            : new Pose3d[] {});
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
