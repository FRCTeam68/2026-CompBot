package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.commands.ShooterCommands;
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

  private static final LoggedNetworkBoolean autonDepot =
      new LoggedNetworkBoolean("SmartDashboard/Auton/Depot", false);

  @SuppressWarnings("unused")
  private static final LoggedNetworkBoolean autonClimb =
      new LoggedNetworkBoolean("SmartDashboard/Auton/Climb", false);

  private static final LoggedNetworkBoolean autonOutpost =
      new LoggedNetworkBoolean("SmartDashboard/Auton/Outpost", false);

  public static enum StartingPose {
    Left,
    Left1,
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
    autonStartingPose.addOption("Left1", Auton.StartingPose.Left1);
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

  public static Command SelectedCommand() {
    if (autonStartingPose.get() == null) {
      return Commands.none();
    }

    switch (autonStartingPose.get()) {
      case Left:
        return Apollo();

      case Left1:
        return Apollo1();

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
              Command myCommand1;
              Command myCommand2 = Commands.none();
              if (autonDepot.get()) {

                myCommand1 =
                    Commands.sequence(
                        Commands.parallel(
                            PathUtil.followPath("Center Depot"),
                            Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeOn())),
                        IntakeCommands.stop(),
                        PathUtil.followPath("Depot Tower"),
                        Commands.waitSeconds(
                            3.0) // replace with ShootCommands.Shootloop.withTimeOut(3.0.)
                        );

                if (autonOutpost.get()) {
                  myCommand2 = PathUtil.followPath("Tower Outpost");
                  myCommand2 =
                      myCommand2.andThen(
                          Commands.waitSeconds(
                              3.0)); // replace with ShootCommands.Shootloop.withTimeOut(3.0.)
                }
              } else {
                myCommand1 =
                    Commands.sequence(
                        Commands.parallel(
                            PathUtil.followPath("Center Outpost"),
                            Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeOn())),
                        IntakeCommands.stop(),
                        PathUtil.followPath("Outpost Tower"),
                        Commands.waitSeconds(
                            3.0) // replace with ShootCommands.Shootloop.withTimeOut(3.0.)
                        );
              }

              return myCommand1.andThen(myCommand2);
            },
            Set.of(drive))
        .withName("Auton_Terra");
  }

  private static Command Apollo() {
    return new DeferredCommand(
            () -> {
              Command myCommand1;
              Command myCommand2;

              myCommand1 =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Left Trench A"),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeOn())),
                      IntakeCommands.stop(),
                      PathUtil.followPath("Left Trench B"),
                      Commands.waitSeconds(
                          3.0) // replace with ShootCommands.Shootloop.withTimeOut(3.0.)
                      );

              if (autonDepot.get()) {
                myCommand2 = PathUtil.followPath("Left Depot");
                myCommand2 =
                    myCommand2.andThen(
                        Commands.waitSeconds(
                            3.0)); // replace with ShootCommands.Shootloop.withTimeOut(3.0.)
              } else {
                myCommand2 = PathUtil.followPath("Left Free Seconds");
              }

              return myCommand1.andThen(myCommand2);
            },
            Set.of(drive))
        .withName("Auton_Apollo");
  }

  private static Command Apollo1() {
    return new DeferredCommand(
            () -> {
              Command myCommand1;
              Command myCommand2;

              myCommand1 =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Left Trench A1"),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeOn())),
                      PathUtil.followPath("Left Trench A2"),
                      IntakeCommands.stop(),
                      PathUtil.followPath("Left Trench B1"),
                      ShooterCommands.shootAutomatic().withTimeout(3.0));

              if (autonDepot.get()) {
                myCommand2 =
                    Commands.sequence(
                        PathUtil.followPath("Left Depot1"),
                        Commands.parallel(
                            PathUtil.followPath("Left Depot2"),
                            IntakeCommands.intakeOn(),
                            ShooterCommands.shootAutomatic().withTimeout(5.0)),
                        IntakeCommands.stop());
              } else {
                myCommand2 =
                    Commands.sequence(
                        Commands.parallel(
                            PathUtil.followPath("Left Trench C1"),
                            Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeOn())),
                        IntakeCommands.stop(),
                        PathUtil.followPath("Left Trench D1"),
                        ShooterCommands.shootAutomatic().withTimeout(5.0),
                        IntakeCommands.stop());
              }

              return myCommand1.andThen(myCommand2);
            },
            Set.of(drive))
        .withName("Auton_Apollo1");
  }

  private static Command Neptune() {
    return new DeferredCommand(
            () -> {
              Command myCommand1;
              Command myCommand2;

              myCommand1 =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Right Trench A"),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeOn())),
                      IntakeCommands.stop(),
                      PathUtil.followPath("Right Trench B"),
                      Commands.waitSeconds(
                          3.0) // replace with ShootCommands.Shootloop.withTimeOut(3.0.)
                      );

              if (autonOutpost.get()) {
                myCommand2 = PathUtil.followPath("Right Outpost");
                myCommand2 =
                    myCommand2.andThen(
                        Commands.waitSeconds(
                            3.0)); // replace with ShootCommands.Shootloop.withTimeOut(3.0.)
              } else {
                myCommand2 = PathUtil.followPath("Right Free Seconds");
              }

              return myCommand1.andThen(myCommand2);
            },
            Set.of(drive))
        .withName("Auton_Neptune");
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
        return AllianceFlipUtil.apply(PathUtil.getStartingPose("Left Trench A"));

      case Left1:
        return AllianceFlipUtil.apply(PathUtil.getStartingPose("Left Trench A1"));

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
      drive.setPose(getSelectedStartPose());
    }
  }
}
