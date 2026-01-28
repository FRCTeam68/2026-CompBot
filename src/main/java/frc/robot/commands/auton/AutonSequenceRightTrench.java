package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutonUtil;
import java.util.ArrayList;
import java.util.List;

public class AutonSequenceRightTrench implements AutonSequence {
  private static final List<String> pathNames = new ArrayList<>();

  public AutonSequenceRightTrench() {
    for (Path path : Path.values()) {
      pathNames.add(path.getPathName());
    }
  }

  private enum Path {
    RA("Right Trench A"),
    RB("Right Trench B"),
    RC("Right Trench C"),
    RD("Right Trench C 2");

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
  public Command sequence(Drive drive) {
    return Commands.sequence(
        // Shoot preload note
        AutonUtil.followPath(Path.RA.getPathName()),
        AutonUtil.followPath(Path.RB.getPathName()),
        AutonUtil.followPath(Path.RC.getPathName()));
  }
}
