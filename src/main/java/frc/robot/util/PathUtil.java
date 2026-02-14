package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class PathUtil {
  public static PathPlannerPath getPath(String name) {
    try {
      return PathPlannerPath.fromPathFile(name);
    } catch (Exception e) {
      System.out.print("Error getting path with name \"" + name + "\": ");
      e.printStackTrace();
      return new PathPlannerPath(
          PathPlannerPath.waypointsFromPoses(Pose2d.kZero, Pose2d.kZero, Pose2d.kZero),
          PathConstraints.unlimitedConstraints(12),
          new IdealStartingState(0.0, Rotation2d.kZero),
          new GoalEndState(0.0, Rotation2d.kZero));
    }
  }

  public static Pose2d getStartingPose(String name) {
    return getStartingPose(getPath(name));
  }

  /**
   * Get the starting pose from the first loaded path.
   *
   * @return The starting pose for the first path.
   *     <li>If an error occurs, this will return a pose at the origin facing toward the positive X
   *         axis.
   */
  public static Pose2d getStartingPose(PathPlannerPath path) {
    try {
      // TODO: test if this needs to be flipped
      return path.getStartingHolonomicPose().get();
    } catch (Exception e) {
      System.out.print("Error getting starting holonomic pose from path \"" + path.name + "\": ");
      e.printStackTrace();
    }

    return Pose2d.kZero;
  }

  public static Pose2d getEndPose(String name) {
    return getEndPose(getPath(name));
  }

  /**
   * Get the starting pose from the first loaded path.
   *
   * @return The starting pose for the first path.
   *     <li>If an error occurs, this will return a pose at the origin facing toward the positive X
   *         axis.
   */
  public static Pose2d getEndPose(PathPlannerPath path) {
    try {
      // TODO: test if this needs to be flipped
      List<Pose2d> poses = path.getPathPoses();
      return poses.get(poses.size() - 1);
    } catch (Exception e) {
      System.out.print("Error getting ending holonomic pose from path \"" + path.name + "\": ");
      e.printStackTrace();
    }

    return Pose2d.kZero;
  }

  public static Command followPath(String name) {
    return followPath(getPath(name));
  }

  /**
   * Builds a command to follow a path
   *
   * @param path The path to follow
   * @return A path following command for the given path
   *     <li>If an error occurs, this will return a command that does nothing, finishing
   *         immediately.
   */
  public static Command followPath(PathPlannerPath path) {
    try {
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      System.out.print("Error following path: ");
      e.printStackTrace();
      return Commands.none();
    }
  }
}
