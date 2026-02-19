package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotSystem;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PathUtil;
import frc.robot.util.geometry.AllianceFlipUtil;
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
    Right,
    Center
  }

  public static enum Special {
    None,
    Minimal,
    Full,
    Fast
  }

  private static final Alert noAutoSelectedAlert =
      new Alert("No autonomous routine selected.", AlertType.kError);
  private static final Alert startingPoseAlert =
      new Alert(
          "Current robot pose does not match the starting pose for selected auton. Possible causes include the incorrect auton is selected, the camera is not getting a clear view of an april tag, or the robot is in the wrong location.",
          AlertType.kError);

  public static void initDashboardInputs() {
    // Configure starting pose
    autonStartingPose.addOption("Left", Auton.StartingPose.Left);
    autonStartingPose.addOption("Center", Auton.StartingPose.Center);
    autonStartingPose.addOption("Right", Auton.StartingPose.Right);

    // Configure special
    autonSpecial.addDefaultOption("None", Auton.Special.None);
    autonSpecial.addOption("Minimal", Auton.Special.Minimal);
    autonSpecial.addOption("Full", Auton.Special.Full);
    autonSpecial.addOption("Fast", Auton.Special.Fast);
  }

  public static void UpdateAlerts() {
    if (DriverStation.isAutonomous()) {
      noAutoSelectedAlert.set(autonStartingPose.get() == null);

      startingPoseAlert.set(
          autonStartingPose.get() != null
              && (getSelectedStartPose().minus(drive.getPose()).getTranslation().getNorm() > 0.25
                  || getSelectedStartPose().minus(drive.getPose()).getRotation().getDegrees()
                      > 20));
    } else {
      noAutoSelectedAlert.set(false);
      startingPoseAlert.set(false);
    }
  }

  public static Command command() {
    if (autonStartingPose.get() == null) {
      return Commands.none();
    }

    switch (autonStartingPose.get()) {
      case Left:
        return Commands.none();

      case Center:
        return Terra();

      case Right:
        return Neptune();

      default:
        return Commands.none();
    }
  }

  private static Command Terra() {
    return new DeferredCommand(
        () -> {
          Command myCommand;
          if (autonDepot.get()) {
            myCommand = (IntakeCommands.intakeOn());
            myCommand = myCommand.andThen(PathUtil.followPath("Center Depot"));
            myCommand = myCommand.andThen(PathUtil.followPath("Depot Tower"));
            if (autonOutpost.get()) {
              myCommand = myCommand.andThen(PathUtil.followPath("Tower Outpost"));
            }
          } else {
            myCommand = (IntakeCommands.intakeOn());
            myCommand = myCommand.andThen(PathUtil.followPath("Center Outpost"));
            myCommand = myCommand.andThen(PathUtil.followPath("Outpost Tower"));
          }
          return myCommand;
        },
        Set.of(drive));
  }

  private static Command Neptune() {
    return new DeferredCommand(
        () -> {
          Command myCommand;
          myCommand = (IntakeCommands.intakeOn());
          myCommand = myCommand.andThen(PathUtil.followPath("Right Trench A"));
          myCommand = myCommand.andThen(PathUtil.followPath("Right Trench B"));
          if (autonOutpost.get()) {
            myCommand = myCommand.andThen(PathUtil.followPath("Right Outpost"));
          } else {
            myCommand = myCommand.andThen(PathUtil.followPath("Right Free Seconds"));
          }
          return myCommand;
        },
        Set.of(drive));
  }

  @SuppressWarnings("unused")
  private static Command neutralZone() {
    return Commands.none();
  }

  public static Pose2d getSelectedStartPose() {
    if (autonStartingPose.get() == null) {
      return AllianceFlipUtil.apply(Pose2d.kZero);
    }

    switch (autonStartingPose.get()) {
      case Left:
        // TODO: AUTON - update to use first left path when left paths added
        return AllianceFlipUtil.apply(new Pose2d(4.0, 7.5, Rotation2d.kZero));

      case Center:
        return AllianceFlipUtil.apply(PathUtil.getStartingPose("Center Depot"));

      case Right:
        return AllianceFlipUtil.apply(PathUtil.getStartingPose("Right Trench A"));

      default:
        return AllianceFlipUtil.apply(Pose2d.kZero);
    }
  }

  /** load starting pose if simulator is running */
  public static void loadStartPoseSim() {
    if (Constants.getMode() == Mode.SIM) {
      // TODO: AUTON - update to use first left path when left paths added
      drive.setPose(getSelectedStartPose());
    }
  }
}
