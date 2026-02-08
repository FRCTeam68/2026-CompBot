package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotSystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutonUtil;
import java.util.ArrayList;
import java.util.List;

public class AutonSequenceCenter implements AutonSequence {
  // Complete system
  private static final RobotSystem system = RobotSystem.getInstance();

  // Subsystems
  private static final Drive drive = system.getDrive();

  private static final List<String> pathNames = new ArrayList<>();

  public AutonSequenceCenter() {
    for (Path path : Path.values()) {
      pathNames.add(path.getPathName());
    }
  }

  private enum Path {
    DA("Middle Depot A"),
    DB("Middle Depot B"),
    DC("Middle Depot C"),
    DD("Middle Depot D");

    private String pathName;

    private Path(String pathName) {
      this.pathName = pathName;
    }

    private String getPathName() {
      return pathName;
    }
  }

  @Override
  public List<String> getPathNames() {
    return pathNames;
  }

  @Override
  public Command sequence() {
    return Commands.sequence(
        // Shoot preload note
        AutonUtil.followPath(Path.DA.getPathName()),
        AutonUtil.followPath(Path.DB.getPathName()),
        AutonUtil.followPath(Path.DC.getPathName()),
        AutonUtil.followPath(Path.DD.getPathName()));
  }
}
