package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.FieldConstants;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShotVisualizer {
  private static final Transform3d launcherTransform =
      new Transform3d(0.24, 0, 2.17, new Rotation3d());
  private static final double shotSpeed = 3.5; // Meters per sec
  private static final double shotPitch = Units.degreesToRadians(50); // Degrees from horizontal
  private static final double gravity = 9.8; // Meters per sec²
  private static final double targetHeight = 2.2; // Meters: height from ground to end visualization

  private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

  private static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);
  private static final Translation3d redSpeaker = new Translation3d(16.317, 5.55, 2.1);

  public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    robotPoseSupplier = supplier;
  }

  public static Command shoot() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  final Pose3d startPose =
                      new Pose3d(robotPoseSupplier.get()).transformBy(launcherTransform);
                  final boolean isRed =
                      DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Red);
                  final Pose3d endPose =
                      new Pose3d(isRed ? redSpeaker : blueSpeaker, startPose.getRotation());

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () -> {
                            Logger.recordOutput(
                                "NoteVisualizer",
                                new Pose3d[] {
                                  startPose.interpolate(endPose, timer.get() / duration)
                                });
                          })
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> {
                            Logger.recordOutput("NoteVisualizer", new Pose3d[] {});
                          });
                },
                Set.of())
            .ignoringDisable(true));
  }

  public static Command shootParabula() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  final Pose3d startPose =
                      new Pose3d(robotPoseSupplier.get()).transformBy(launcherTransform);
                  final double shotYaw =
                      startPose.getRotation().getZ()
                          + Math.PI; // Degrees: 0 is away from blue drive station
                  final Timer timer = new Timer();
                  final double Vx = shotSpeed * Math.cos(shotPitch) * Math.cos(shotYaw);
                  final double Vy = shotSpeed * Math.cos(shotPitch) * Math.sin(shotYaw);
                  final double Vz0 = shotSpeed * Math.sin(shotPitch);
                  final double duration =
                      getDurationParabula(startPose, Vx, Vy, Vz0, targetHeight, EndType.falling);

                  return Commands.sequence(
                      Commands.waitSeconds(0.15),
                      Commands.runOnce(() -> timer.start()),
                      Commands.run(
                              () -> {
                                Logger.recordOutput(
                                    "ShotVisualizer",
                                    new Pose3d[] {
                                      new Pose3d(
                                          startPose.getX() + (Vx * timer.get()),
                                          startPose.getY() + (Vy * timer.get()),
                                          startPose.getZ()
                                              + ((Vz0 * timer.get())
                                                  - (0.5 * gravity * Math.pow(timer.get(), 2))),
                                          startPose.getRotation())
                                    });
                              })
                          .until(() -> timer.hasElapsed(duration))
                          .finallyDo(
                              () -> {
                                Logger.recordOutput("ShotVisualizer", new Pose3d[] {});
                              }));
                },
                Set.of())
            .ignoringDisable(true));
  }

  private static enum EndType {
    perimeter,

    rising,

    falling
  }

  private static double getDurationParabula(
      Pose3d startPose, double Vx, double Vy, double Vz0, double targetHeight, EndType endType) {
    // If starting height is less then 0
    // Return 0
    if (startPose.getZ() < 0) return 0;

    Double duration = null;
    Double duration0H = null;

    // Calculate duration for 0 target height
    duration0H =
        (Vz0 + Math.sqrt((2 * gravity * startPose.getZ()) - (2 * gravity * 0) + Math.pow(Vz0, 2)))
            / gravity;

    // Calculate duration for specified end type
    switch (endType) {
      case perimeter:
        Double durationX = null;
        Double durationY = null;
        if (Vx != 0)
          durationX =
              (Vx > 0)
                  ? (FieldConstants.fieldLength - startPose.getX()) / Vx
                  : (0 - startPose.getX()) / Vx;
        if (Vy != 0)
          durationY =
              (Vy > 0)
                  ? (FieldConstants.fieldWidth - startPose.getY()) / Vy
                  : (0 - startPose.getY()) / Vy;

        if (durationX == null && durationY == null) {
          duration = null;
        } else if (durationX != null && durationY == null) {
          duration = durationX;
        } else if (durationX == null && durationY != null) {
          duration = durationY;
        } else {
          duration = Math.min(durationX, durationY);
        }
        break;

      case rising:
        duration =
            (Vz0
                    - Math.sqrt(
                        (2 * gravity * startPose.getZ())
                            - (2 * gravity * targetHeight)
                            + Math.pow(Vz0, 2)))
                / gravity;
        break;

      case falling:
        duration =
            (Vz0
                    + Math.sqrt(
                        (2 * gravity * startPose.getZ())
                            - (2 * gravity * targetHeight)
                            + Math.pow(Vz0, 2)))
                / gravity;
        break;
    }

    // Return lowest non null duration value
    return (duration != null && duration < duration0H) ? duration : duration0H;
  }
}
