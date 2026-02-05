package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.System;
import frc.robot.subsystems.drive.Drive;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShotVisualizer {
  private static final Translation3d shooterPosition =
      new Translation3d(-0.160018476, 0.1335875408, 0.4431027206); // robot relative
  private static final double stepSecs = 0.04;
  private static final double FlywheelDiameter = 4.0;
  private static final double gravity = 9.8;

  // Complete system
  private static final System system = System.getInstance();

  // Subsystems
  private static final Drive drive = system.getDrive();
  private static final Shooter shooter = system.getShooter();

  private static final Supplier<Pose2d> robotPoseSupplier = drive::getPose;
  private static final Supplier<Double> flywheelVelocitySupplier = () -> 0.0;
  private static final Supplier<Double> hoodElevationSupplier = () -> 0.0;
  private static final Supplier<Rotation2d> turretAngleSupplier = () -> new Rotation2d();
  private static final Supplier<Double> feederSetpointSupplier = () -> 0.0;

  public static void visualize() {
    List<Pose3d> trajectory = new LinkedList<>();

    if (feederSetpointSupplier.get() > 0.0) {
      Pose3d initialPose =
          new Pose3d(
              shooterPosition
                  .rotateBy(new Rotation3d(robotPoseSupplier.get().getRotation()))
                  .plus(new Translation3d(robotPoseSupplier.get().getTranslation())),
              new Rotation3d(
                  0.0,
                  0.0,
                  turretAngleSupplier.get().getRadians()
                      + robotPoseSupplier.get().getRotation().getRadians()
                      + Math.PI));

      Translation2d initialVelocity =
          new Translation2d(
              Units.inchesToMeters(flywheelVelocitySupplier.get() * FlywheelDiameter * Math.PI),
              new Rotation2d(Units.degreesToRadians(hoodElevationSupplier.get())));

      double time = 0;

      while (trajectory.size() == 0 || trajectory.get(trajectory.size() - 1).getZ() > 0.0) {
        trajectory.add(
            initialPose.transformBy(
                new Transform3d(
                    initialVelocity.getX() * time,
                    0.0,
                    (initialVelocity.getY() * time) - (0.5 * gravity * Math.pow(time, 2)),
                    new Rotation3d())));
        time += stepSecs;
      }

      trajectory.remove(trajectory.size() - 1);
    }

    Logger.recordOutput("Shooter/ShotVisualizer", trajectory.toArray(new Pose3d[0]));
  }
}
