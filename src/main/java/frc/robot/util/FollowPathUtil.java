package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;

public class FollowPathUtil {
  @Getter private static List<PathPlannerPath> path = new ArrayList<>();
  private static List<String> loadedPathNames = new ArrayList<>();
  private static final Alert loadPathAlert =
      new Alert("Error loading teleop paths.", AlertType.kError);

  public static enum TeleopPath {}

  /**
   * Loads paths from storage. This method can be safely be called periodically.
   *
   * @param pathNames List of path names
   */
  public static void loadPathFromList(List<String> pathNames) {
    if (pathNames == loadedPathNames) {
      return;
    }

    if (pathNames == null) {
      path.clear();
      loadedPathNames = pathNames;
      return;
    }

    boolean pathLoadError = false;

    for (int i = 0; i < pathNames.size(); i++) {
      try {
        if (pathNames.get(i).toLowerCase().contains("group")) {
          path.addAll(PathPlannerAuto.getPathGroupFromAutoFile(pathNames.get(i)));
        } else {
          path.add(PathPlannerPath.fromPathFile(pathNames.get(i)));
        }
      } catch (Exception e) {
        System.out.print("Error loading path from list: ");
        e.printStackTrace();
        pathLoadError = true;
      }
    }

    loadPathAlert.set(pathLoadError);
    loadedPathNames = pathNames;
  }

  /**
   * Get the starting pose from the first loaded path.
   *
   * @return The starting pose for the first path.
   *     <li>If an error occurs, this will return a pose at the origin facing toward the positive X
   *         axis.
   */
  public static Pose2d getStartingPose() {
    if (path.size() > 0) {
      try {
        return AllianceFlipUtil.apply(path.get(0).getStartingHolonomicPose().get());
      } catch (Exception e) {
        System.out.print("Error getting starting holonomic pose: ");
        e.printStackTrace();
      }
    } else {
      System.out.println("Error getting starting pose: Path not initialized.");
    }

    return new Pose2d();
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
      // This is a work around since PathPlanner only uses the newest field size and flip type.
      if (AllianceFlipUtil.shouldFlip()) {
        path = flipPath(path);
      }

      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      System.out.print("Error following path: ");
      e.printStackTrace();
      return Commands.none();
    }
  }

  /** This is a work around since PathPlanner only uses the newest field size and flip type. */
  private static PathPlannerPath flipPath(PathPlannerPath path) {
    List<Waypoint> waypointsFlipped = new ArrayList<>();
    List<RotationTarget> rotationTargetsFlipped = new ArrayList<>();
    List<PointTowardsZone> pointTowardsZonesFlipped = new ArrayList<>();

    for (int i = 0; i < path.getWaypoints().size(); i++) {
      Waypoint waypoint = path.getWaypoints().get(i);
      waypointsFlipped.add(
          new Waypoint(
              (waypoint.prevControl() != null)
                  ? AllianceFlipUtil.apply(waypoint.prevControl())
                  : null,
              AllianceFlipUtil.apply(waypoint.anchor()),
              (waypoint.nextControl() != null)
                  ? AllianceFlipUtil.apply(waypoint.nextControl())
                  : null));
    }
    for (int i = 0; i < path.getRotationTargets().size(); i++) {
      RotationTarget rotationTarget = path.getRotationTargets().get(i);
      rotationTargetsFlipped.add(
          new RotationTarget(
              rotationTarget.position(), AllianceFlipUtil.apply(rotationTarget.rotation())));
    }
    for (int i = 0; i < path.getPointTowardsZones().size(); i++) {
      PointTowardsZone pointTowardsZone = path.getPointTowardsZones().get(i);
      pointTowardsZonesFlipped.add(
          new PointTowardsZone(
              pointTowardsZone.name(),
              AllianceFlipUtil.apply(pointTowardsZone.targetPosition()),
              AllianceFlipUtil.apply(pointTowardsZone.rotationOffset()),
              pointTowardsZone.minPosition(),
              pointTowardsZone.maxPosition()));
    }

    path =
        new PathPlannerPath(
            waypointsFlipped,
            rotationTargetsFlipped,
            pointTowardsZonesFlipped,
            path.getConstraintZones(),
            path.getEventMarkers(),
            path.getGlobalConstraints(),
            new IdealStartingState(
                path.getIdealStartingState().velocityMPS(),
                AllianceFlipUtil.apply(path.getIdealStartingState().rotation())),
            new GoalEndState(
                path.getGoalEndState().velocityMPS(),
                AllianceFlipUtil.apply(path.getGoalEndState().rotation())),
            false);

    path.preventFlipping = true;

    return path;
  }
}
