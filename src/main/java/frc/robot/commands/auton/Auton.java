package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotSystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PathUtil;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Auton {
  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final Drive drive = robotSystem.getDrive();

  // Dashboard inputs
  private static final LoggedDashboardChooser<Auton.StartingPose> autonStartingPose =
      new LoggedDashboardChooser<>("Auton/StartingPose");

  @SuppressWarnings("unused")
  private static final LoggedNetworkBoolean autonFullField =
      new LoggedNetworkBoolean("SmartDashboard/Auton/FullField", false);

  private static final LoggedDashboardChooser<Auton.Special> autonSpecial =
      new LoggedDashboardChooser<>("Auton/Special");

  @SuppressWarnings("unused")
  private static final LoggedNetworkBoolean autonDepot =
      new LoggedNetworkBoolean("SmartDashboard/Auton/Depot", false);

  private static final LoggedNetworkBoolean autonClimb =
      new LoggedNetworkBoolean("SmartDashboard/Auton/Climb", false);

  private static final LoggedNetworkBoolean autonOutpost =
      new LoggedNetworkBoolean("SmartDashboard/Auton/Outpost", false);

  public static enum StartingPose {
    Left,
    CenterLeft,
    CenterRight,
    Right,
    Center,
    RightJ
  }

  public static enum Special {
    None,
    Minimal,
    Full,
    Fast
  }

  public static void initDashboardInputs() {
    // Configure starting pose
    autonStartingPose.addDefaultOption("Left", Auton.StartingPose.Left);
    autonStartingPose.addOption("CenterLeft", Auton.StartingPose.CenterLeft);
    autonStartingPose.addOption("Center", Auton.StartingPose.Center);
    autonStartingPose.addOption("CenterRight", Auton.StartingPose.CenterRight);
    autonStartingPose.addOption("Right", Auton.StartingPose.Right);
    autonStartingPose.addOption("RightJ", Auton.StartingPose.RightJ);

    // Configure special
    autonSpecial.addDefaultOption("None", Auton.Special.None);
    autonSpecial.addOption("Minimal", Auton.Special.Minimal);
    autonSpecial.addOption("Full", Auton.Special.Full);
    autonSpecial.addOption("Fast", Auton.Special.Fast);
  }

  public static Command command() {
    if (Constants.getMode() == Mode.SIM) {
      // drive.setPose(AutonUtil.getStartingPose());
    }

    switch (autonStartingPose.get()) {
      case Left:
        return Commands.none();

      case CenterLeft:
        if (Constants.getMode() == Mode.SIM) {
          drive.setPose(PathUtil.getStartingPose("Middle Depot A"));
        }
        return Water();

      case Center:
        return Terra();

      case CenterRight:
        return Marz();

      case Right:
        return Neptune();

      case RightJ:
        return RightJ();

      default:
        return Commands.none();
    }
  }

  private static Command Terra() {
    return new DeferredCommand(
        () -> {
          Command myCommand;
          if (autonDepot.get()) {
            myCommand = PathUtil.followPath("Center Depot");
            myCommand = myCommand.andThen(PathUtil.followPath("Depot Tower"));
            if (autonOutpost.get()) {
              myCommand = myCommand.andThen(PathUtil.followPath("Tower Outpost"));
            }
          } else {
            myCommand = PathUtil.followPath("Center Outpost");
            myCommand = myCommand.andThen(PathUtil.followPath("Outpost Tower"));
          }
          return myCommand;
        },
        Set.of(drive));
  }

  private static Command Marz() {
    return Commands.sequence(
        // Shoot preload note
        PathUtil.followPath("Right Trench A"),
        PathUtil.followPath("Right Trench B"),
        PathUtil.followPath("Right Trench C"));
  }

  private static Command Neptune() {
    return Commands.sequence(
        // Shoot preload note
        PathUtil.followPath("Right Trench A"),
        PathUtil.followPath("Right Trench B"),
        PathUtil.followPath("Right Trench C 2"));
  }

  @SuppressWarnings("unused")
  private static Command neutralZone() {
    return Commands.none();
  }

  private static Command Water() {
    return Commands.sequence(
        // Shoot preload note
        PathUtil.followPath("Middle Depot A"),
        PathUtil.followPath("Middle Depot B"),
        PathUtil.followPath("Middle Depot C"),
        PathUtil.followPath("Middle Depot D"));
  }

  public static Command RightJ() {
    if (Constants.getMode() == Mode.SIM) {
      drive.setPose(PathUtil.getStartingPose("Right Trench A"));
    }
    return new DeferredCommand(
        () -> {
          // initialization
          Command command =
              Commands.sequence(
                  // Shoot preload note
                  PathUtil.followPath("Right Trench A"), PathUtil.followPath("Right Trench B"));

          if (autonClimb.get()) {
            command = command.andThen(PathUtil.followPath("Right Trench C 2"));
          } else {
            command = command.andThen(PathUtil.followPath("Right Trench C"));
          }
          return command;
        },
        Set.of(drive));
  }

  /**
   * Creates a an auton command with the supplied sequence.
   * <li>If in simulation, the robot pose is set to the inital pose of the first path.
   *
   * @return Auton command
   *     <li>If config or config.sequence is null, this will return null.
   */
  // public static Command autonCommand(AutonSequence root) {
  //   if (Constants.getMode() == Mode.SIM) {
  //     drive.setPose(AutonUtil.getStartingPose());
  //   }

  //   if (root == null) root = new AutonSequence() {};
  //   return root.sequence();
  // }
}
