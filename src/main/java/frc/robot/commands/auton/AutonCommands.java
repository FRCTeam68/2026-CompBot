package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotSystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutonUtil;

public class AutonCommands {
  // Complete system
  private static final RobotSystem system = RobotSystem.getInstance();

  // Subsystems
  private static final Drive drive = system.getDrive();

  /**
   * Creates a an auton command with the supplied sequence.
   * <li>If in simulation, the robot pose is set to the inital pose of the first path.
   *
   * @return Auton command
   *     <li>If config or config.sequence is null, this will return null.
   */
  public static Command autonCommand(AutonSequence root) {
    if (Constants.getMode() == Mode.SIM) {
      drive.setPose(AutonUtil.getStartingPose());
    }

    if (root == null) root = new AutonSequence() {};
    return root.sequence();
  }
}
