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
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class PathUtil {
  // Simple cache to avoid re-reading and reparsing path files on first use.
  private static final Map<String, PathPlannerPath> pathCache = new ConcurrentHashMap<>();

  /**
   * Returns the path with the specified name. If no path exists this will return a path that sits
   * at the origin and does nothing.
   */
  public static PathPlannerPath getPath(String name) {
    // Check cache first to avoid file IO on the first runtime call. If loading fails,
    // fall back to a safe no-op path.
    if (pathCache.containsKey(name)) {
      return pathCache.get(name);
    }

    try {
      PathPlannerPath p = PathPlannerPath.fromPathFile(name);
      pathCache.put(name, p);
      return p;
    } catch (Exception e) {
      System.out.print("Error getting path with name \"" + name + "\": ");
      e.printStackTrace();
      PathPlannerPath fallback =
          new PathPlannerPath(
              PathPlannerPath.waypointsFromPoses(Pose2d.kZero, Pose2d.kZero, Pose2d.kZero),
              PathConstraints.unlimitedConstraints(12),
              new IdealStartingState(0.0, Rotation2d.kZero),
              new GoalEndState(0.0, Rotation2d.kZero));
      pathCache.put(name, fallback);
      return fallback;
    }
  }

  /** Preload one or more path files into the cache. Safe to call multiple times. */
  public static void preload(String... names) {
    if (names == null) {
      return;
    }
    for (String n : names) {
      try {
        // getPath will populate the cache and swallow errors.
        getPath(n);
      } catch (Exception e) {
        // ignore - getPath already prints errors
      }
    }
  }

  /**
   * Get the starting pose for the specified path.
   *
   * @param name The name of the path to use.
   * @return The starting pose for the specified path.
   *     <li>If an error occurs, this will return a pose at the origin facing toward the positive X
   *         axis.
   */
  public static Pose2d getStartingPose(String name) {
    return getStartingPose(getPath(name));
  }

  /**
   * Get the starting pose for the specified path.
   *
   * @param name The name of the path to use.
   * @param mirror If the path should be mirrored to the left side.
   * @return The starting pose for the specified path.
   *     <li>If an error occurs, this will return a pose at the origin facing toward the positive X
   *         axis.
   */
  public static Pose2d getStartingPose(String name, boolean mirror) {
    if (mirror) {
      return getStartingPose(getPath(name).mirrorPath());
    } else {
      return getStartingPose(getPath(name));
    }
  }

  /**
   * Get the starting pose for the specified path.
   *
   * @param path The path to use.
   * @return The starting pose for the specified path.
   *     <li>If an error occurs, this will return a pose at the origin facing toward the positive X
   *         axis.
   */
  public static Pose2d getStartingPose(PathPlannerPath path) {
    try {
      return path.getStartingHolonomicPose().get();
    } catch (Exception e) {
      System.out.print("Error getting starting holonomic pose from path \"" + path.name + "\": ");
      e.printStackTrace();
    }

    return Pose2d.kZero;
  }

  /**
   * Get the ending pose for the specified path.
   *
   * @param name The name of the path to use.
   * @return The ending pose for the specified path.
   *     <li>If an error occurs, this will return a pose at the origin facing toward the positive X
   *         axis.
   */
  public static Pose2d getEndPose(String name) {
    return getEndPose(getPath(name));
  }

  /**
   * Get the ending pose for the specified path.
   *
   * @param name The name of the path to use.
   * @param mirror If the path should be mirrored to the left side.
   * @return The ending pose for the specified path.
   *     <li>If an error occurs, this will return a pose at the origin facing toward the positive X
   *         axis.
   */
  public static Pose2d getEndPose(String name, boolean mirror) {
    if (mirror) {
      return getEndPose(getPath(name).mirrorPath());
    } else {
      return getEndPose(getPath(name));
    }
  }

  /**
   * Get the ending pose for the specified path.
   *
   * @param path The path to use.
   * @return The ending pose for the specified path.
   *     <li>If an error occurs, this will return a pose at the origin facing toward the positive X
   *         axis.
   */
  public static Pose2d getEndPose(PathPlannerPath path) {
    try {
      List<Pose2d> poses = path.getPathPoses();
      return poses.get(poses.size() - 1);
    } catch (Exception e) {
      System.out.print("Error getting ending holonomic pose from path \"" + path.name + "\": ");
      e.printStackTrace();
    }

    return Pose2d.kZero;
  }

  /**
   * Builds a command to follow a path.
   *
   * @param name The name of the path to follow.
   * @return A path following command for the given path
   *     <li>If an error occurs, this will return a command that does nothing, finishing
   *         immediately.
   */
  public static Command followPath(String name) {
    return followPath(getPath(name));
  }

  /**
   * Builds a command to follow a path.
   *
   * @param name The name of the path to follow.
   * @param mirror If the path should be mirrored to the left side.
   * @return A path following command for the given path
   *     <li>If an error occurs, this will return a command that does nothing, finishing
   *         immediately.
   */
  public static Command followPath(String name, boolean mirror) {
    if (mirror) {
      return followPath(getPath(name).mirrorPath());
    } else {
      return followPath(getPath(name));
    }
  }

  /**
   * Builds a command to follow a path.
   *
   * @param path The path to follow.
   * @return A path following command for the given path.
   *     <li>If an error occurs, this will return a command that does nothing, finishing
   *         immediately.
   */
  public static Command followPath(PathPlannerPath path) {
    LoggedTracerStatic.record("FollowPathMethod");
    try {
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      System.out.print("Error following path: ");
      e.printStackTrace();
      return Commands.none();
    }
  }
}
