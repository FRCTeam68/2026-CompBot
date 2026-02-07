package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

public interface AutonSequence {
  default List<String> getPathNames() {
    return null;
  }

  default Command sequence() {
    return null;
  }
}
