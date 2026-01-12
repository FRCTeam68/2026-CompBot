package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;

/** Flip field pose if on red alliance */
public class AllianceFlipUtil {
  public static double applyX(double x) {
    return shouldFlip() ? FieldConstants.fieldLength - x : x;
  }

  public static double applyY(double y) {
    return (shouldFlip() && FieldConstants.symmetryType == SymmetryType.Rotated)
        ? FieldConstants.fieldWidth - y
        : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip()
        ? switch (FieldConstants.symmetryType) {
          case Mirrored -> Rotation2d.kPi.rotateBy(rotation.unaryMinus());
          case Rotated -> rotation.rotateBy(Rotation2d.kPi);
        }
        : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static Translation3d apply(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  public static Rotation3d apply(Rotation3d rotation) {
    // TODO: make sure this works as expected
    return shouldFlip()
        ? switch (FieldConstants.symmetryType) {
          case Mirrored ->
              new Rotation3d(rotation.getX(), rotation.getY(), Math.PI)
                  .minus(new Rotation3d(0, 0, rotation.getZ()));
          case Rotated -> rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI));
        }
        : rotation;
  }

  public static Pose3d apply(Pose3d pose) {
    return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

  public static enum SymmetryType {
    /** Field with rotational symmetry. */
    Rotated,

    /** Field with reflectional symmetry. */
    Mirrored
  }
}
