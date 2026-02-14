package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotSystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.shooter.hood.Shooter;

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

  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final Drive drive = robotSystem.getDrive();
  private static final Shooter shooter = robotSystem.getShooter();
  private static final RollerSystem feeder = robotSystem.getFeeder();

  private static final Supplier<Pose2d> robotPoseSupplier = drive::getPose;
  private static final Supplier<ChassisSpeeds> fieldVelocitySupplier = drive::getFieldVelocity;
  private static final Supplier<Double> flywheelVelocitySupplier =
      () -> shooter.getFlywheel().getVelocity();
  private static final Supplier<Double> hoodElevationSupplier =
      () -> shooter.getHood().getPosition();
  private static final Supplier<Rotation2d> turretAngleSupplier = () -> new Rotation2d();
  private static final Supplier<Double> feederSetpointSupplier = feeder::getSetpointVolts;

  public static void visualize() {
    List<Pose3d> trajectory = new LinkedList<>();

    // Only calculate trajectory point if the feeder is running
    if (feederSetpointSupplier.get() > 0.0) {
      double time = 0;

      // All calcuations are field relative
      Translation3d initialPose =
          shooterPosition
              .rotateBy(new Rotation3d(robotPoseSupplier.get().getRotation()))
              .plus(new Translation3d(robotPoseSupplier.get().getTranslation()));

      Translation3d initialVelocity =
          // Shooter velocity component
          new Translation3d(
                  Units.inchesToMeters(
                      flywheelVelocitySupplier.get() * FlywheelDiameter * Math.PI / 2.0),
                  new Rotation3d(
                      0.0,
                      -Units.degreesToRadians(hoodElevationSupplier.get()),
                      turretAngleSupplier.get().getRadians()
                          + robotPoseSupplier.get().getRotation().getRadians()))
              // Chassis translation velocity component
              .plus(
                  new Translation3d(
                      fieldVelocitySupplier.get().vxMetersPerSecond,
                      fieldVelocitySupplier.get().vyMetersPerSecond,
                      0.0))
              // Chassis rotation velocity component
              .plus(
                  new Translation3d(
                      new Translation2d(
                          Units.radiansToRotations(
                                  fieldVelocitySupplier.get().omegaRadiansPerSecond)
                              * 2.0
                              * shooterPosition.toTranslation2d().getNorm()
                              * Math.PI
                              * 4,
                          shooterPosition
                              .toTranslation2d()
                              .getAngle()
                              .rotateBy(robotPoseSupplier.get().getRotation())
                              .rotateBy(Rotation2d.kCCW_90deg))));

      // Loop over trajectory points
      while (trajectory.size() == 0 || trajectory.get(trajectory.size() - 1).getZ() > 0.0) {
        trajectory.add(
            new Pose3d(
                initialPose.plus(
                    new Translation3d(
                        initialVelocity.getX() * time,
                        initialVelocity.getY() * time,
                        (initialVelocity.getZ() * time) - (0.5 * gravity * Math.pow(time, 2)))),
                Rotation3d.kZero));
        time += stepSecs;
      }

      // Remove last point which will always be below the ground
      trajectory.remove(trajectory.size() - 1);
    }

    Logger.recordOutput("Shooter/ShotVisualizer", trajectory.toArray(new Pose3d[0]));
  }
}
