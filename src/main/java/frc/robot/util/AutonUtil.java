package frc.robot.util;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Objects;
import java.util.stream.Collectors;
import lombok.Getter;

public class AutonUtil {
  @Getter private static Map<String, PathPlannerPath> paths = new HashMap<>();
  private static List<String> loadedPathNames = new ArrayList<>();
  private static Pose2d startingPose = null;
  private static final Alert loadStartingPoseAlert =
      new Alert(
          "Error loading auton starting pose. Auton may not run as expected.", AlertType.kError);
  private static final Alert loadPathAlert =
      new Alert("Error loading auton paths. Auton will not run as expected.", AlertType.kError);

  /** Auto group key. Not case sensitive. */
  private static final String autoGroupKey = "group";

  /**
   * Loads paths from storage. This method can be safely be called periodically.
   *
   * @param pathNames List of path names
   */
  public static void loadPaths(List<String> pathNames) {
    // If pathNmaes matches loadedPathNames do nothing and return immediately
    if (Objects.equals(pathNames, loadedPathNames)) {
      return;
    }

    // Reset variables
    paths.clear();
    startingPose = null;
    boolean startingPoseLoadError = false;
    boolean pathLoadError = false;

    // Load paths from storage
    if (pathNames != null && pathNames.size() > 0) {
      for (int i = 0; i < pathNames.size(); i++) {
        try {
          if (pathNames.get(i).toLowerCase().contains(autoGroupKey.toLowerCase())) {
            List<PathPlannerPath> auto = PathPlannerAuto.getPathGroupFromAutoFile(pathNames.get(i));
            paths.putAll(auto.stream().collect(Collectors.toMap(path -> path.name, path -> path)));
            if (i == 0) startingPose = auto.get(0).getStartingHolonomicPose().get();
          } else {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathNames.get(i));
            System.out.println("Path: " + path.name);
            paths.put(path.name, path);
            if (i == 0) startingPose = path.getStartingHolonomicPose().get();
          }
        } catch (NoSuchElementException e) {
          System.out.print("Error getting starting holonomic pose: ");
          e.printStackTrace();
        } catch (Exception e) {
          System.out.print("Error loading auton path: ");
          e.printStackTrace();
          pathLoadError = true;
        }
      }
    }

    // Set alerts
    loadStartingPoseAlert.set(startingPoseLoadError);
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
    if (startingPose != null) {
      return AllianceFlipUtil.apply(startingPose);
    } else {
      System.out.println(
          "Error getting starting pose: Starting pose is null. Using origin pose as fall back.");
      return new Pose2d();
    }
  }

  /**
   * Builds a command to follow a loaded auton path from a path name
   *
   * @param pathName The name of the path to load
   * @return A path following command for the given path
   *     <li>If an error occurs, this will return a command that does nothing, finishing
   *         immediately.
   */
  public static Command followPath(String pathName) {
    if (!paths.containsKey(pathName)) {
      System.out.println("Error loading auton path from map: " + pathName + " not in map.");
      return Commands.none();
    }

    try {
      return FollowPathUtil.followPath(paths.get(pathName));
    } catch (Exception e) {
      System.out.print("Error loading auton path from map: ");
      e.printStackTrace();
      return Commands.none();
    }
  }

  //   private static final LoggedDashboardChooser<AutonConfig> autonChooser =
  //       new LoggedDashboardChooser<>("Auto Chooser");

  //   public static void initChooser() {
  //     autonChooser.addDefaultOption("NONE", null);
  //     autonChooser.addOption(
  //         "Center Close", new AutonConfig(AutonSequence.Center, "Center Start-Close Right"));

  //     File folder = new File("src/main/java/frc/robot/commands/auton");
  //     String interfaceName = "AutonSequence";
  // try {
  //     for (File file : folder.listFiles((dir, name) -> name.endsWith(".java"))) {
  //         String content = Files.readString(file.toPath());

  //         // naive match
  //         if (content.contains("implements " + interfaceName)) {
  //             System.out.println("Name: " + file.ge);
  //         }
  //     }
  // } catch (Exception e) {
  //     // TODO: handle exception
  // }
  //     // try (Stream<Path> paths = Files.list(folder)) {
  //     //   paths.forEach(
  //     //       path -> {
  //     //         System.out.println("Name: " + new path );
  //     //         System.out.println("Path: " + path.toAbsolutePath());
  //     //         System.out.println("Is directory: " + Files.isDirectory(path));
  //     //         System.out.println("-----");
  //     //       });
  //     // }
  //     // File[] files = Paths.get("robot/commands/auton").;
  //     // for (File file : files) {
  //     //   if (file.getName().equals("AutonSequenceSide.java"))
  //     //     autonChooser.addOption(
  //     //         "test", new AutonConfig(AutonSequence.Center, "Center Start-Close Right"));
  //     // }
  //     // catch (IOException e) {
  //     //   // TODO Auto-generated catch block
  //     //   e.printStackTrace();
  //     // }
  //   }
}
