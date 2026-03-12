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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.PathUtil;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Auton {
  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final Drive drive = robotSystem.getDrive();
  private static final Shooter shooter = robotSystem.getShooter();

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
    Right,
    Center
  }

  public static enum Special {
    None,
    Left_LigerBot,
    Left_Tune_PP_3M_slow,
    Left_Tune_PP_3M_fast,
    Right_Tune_PP_3M_slow,
    Right_1768
  }

  private static final Alert noAutoSelectedAlert =
      new Alert("No autonomous routine selected.", AlertType.kError);
  private static final Alert startingPoseAlert =
      new Alert(
          "Current robot pose does not match the starting pose for selected auton. Possible causes include the incorrect auton is selected, the camera is not getting a clear view of an april tag, or the robot is in the wrong location.",
          AlertType.kError);
  private static final Alert hoodStartingPoseAlert =
      new Alert(
          "The hood is not in the correct starting position. Ensure hood is it maximum elevation and then click \"Zero Hood\" in the tuning tab.",
          AlertType.kError);

  public static void initDashboardInputs() {
    // Configure starting pose
    autonStartingPose.addOption("Left", Auton.StartingPose.Left);
    autonStartingPose.addOption("Center", Auton.StartingPose.Center);
    autonStartingPose.addOption("Right", Auton.StartingPose.Right);

    // Configure special
    autonSpecial.addDefaultOption("None", Auton.Special.None);
    autonSpecial.addOption("Left_LigerBot", Auton.Special.Left_LigerBot);
    autonSpecial.addOption("Right_1768", Auton.Special.Right_1768);
    autonSpecial.addOption("Left_Tune_PP_3M_fast", Auton.Special.Left_Tune_PP_3M_fast);
    autonSpecial.addOption("Left_Tune_PP_3M_slow", Auton.Special.Left_Tune_PP_3M_slow);
    autonSpecial.addOption("Right_Tune_PP_3M_slow", Auton.Special.Right_Tune_PP_3M_slow);
  }

  public static void UpdateAlerts() {
    if (DriverStation.isAutonomous()) {
      noAutoSelectedAlert.set(autonStartingPose.get() == null);
      startingPoseAlert.set(
          autonStartingPose.get() != null
              && (getSelectedStartPose().minus(drive.getPose()).getTranslation().getNorm() > 0.25
                  || getSelectedStartPose().minus(drive.getPose()).getRotation().getDegrees()
                      > 20));
      hoodStartingPoseAlert.set(
          Math.abs(shooter.getHood().getElevation() - Hood.getMaximum()) < 2.0);
    } else {
      noAutoSelectedAlert.set(false);
      startingPoseAlert.set(false);
      hoodStartingPoseAlert.set(false);
    }
  }

  public static Command SelectedCommand() {
    if (autonStartingPose.get() == null) {
      return Commands.none();
    }

    switch (autonStartingPose.get()) {
      case Left:
        switch (autonSpecial.get()) {
          case Left_Tune_PP_3M_slow:
            return PathUtil.followPath("Left_Tune_PP_3M_Forward_slow")
                .andThen(Commands.waitSeconds(3))
                .andThen(PathUtil.followPath("Left_Tune_PP_3M_Back_slow"));
          case Left_Tune_PP_3M_fast:
            return PathUtil.followPath("Left_Tune_PP_3M_Forward_fast")
                .andThen(Commands.waitSeconds(3))
                .andThen(PathUtil.followPath("Left_Tune_PP_3M_Back_fast"));
          case Left_LigerBot:
            return Left_LigerBot();
          case None:
          default:
            return Apollo();
        }

      case Center:
        return Terra();

      case Right:
        switch (autonSpecial.get()) {
          case Right_Tune_PP_3M_slow:
            return PathUtil.followPath("Right_Tune_PP_3M_Forward_slow")
                .andThen(Commands.waitSeconds(3))
                .andThen(PathUtil.followPath("Right_Tune_PP_3M_Back_slow"));
          case Right_1768:
            return Right_1768();
          case None:
          default:
            return Neptune();
        }

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
                        PathUtil.followPath("Center Depot"),
                        // Commands.parallel(
                        //     PathUtil.followPath("Center Depot"),
                        //     Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeRemainOn())),
                        // IntakeCommands.stop(),
                        ShooterCommands.shoot(false).repeatedly().withTimeout(5.0),
                        PathUtil.followPath("Depot Tower"));

                if (autonOutpost.get()) {
                  myCommand2 = PathUtil.followPath("Tower Outpost");
                  myCommand2 =
                      myCommand2.andThen(
                          ShooterCommands.shoot(false).repeatedly().withTimeout(5.0));
                }
              } else {
                myCommand1 =
                    Commands.sequence(
                        PathUtil.followPath("Center Outpost"),
                        ShooterCommands.shoot(false).repeatedly().withTimeout(5.0));
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
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      IntakeCommands.stopSpin(),
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

  private static Command Left_LigerBot() {
    return new DeferredCommand(
            () -> {
              Command myCommand1;
              Command myCommand2;

              myCommand1 =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Left Trench A1"),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      PathUtil.followPath("Left Trench A2"),
                      IntakeCommands.stopSpin(),
                      PathUtil.followPath("Left Trench B1"),
                      ShooterCommands.shoot(false).withTimeout(3.0));

              if (autonDepot.get()) {
                myCommand2 =
                    Commands.sequence(
                        PathUtil.followPath("Left Depot1"),
                        Commands.parallel(
                            PathUtil.followPath("Left Depot2"),
                            IntakeCommands.intakeStatic(false),
                            ShooterCommands.shoot(false).withTimeout(5.0)),
                        IntakeCommands.stopSpin());
              } else {
                myCommand2 =
                    Commands.sequence(
                        Commands.parallel(
                            PathUtil.followPath("Left Trench C1"),
                            Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                        IntakeCommands.stopSpin(),
                        PathUtil.followPath("Left Trench D1"),
                        ShooterCommands.shoot(false).withTimeout(5.0),
                        IntakeCommands.stopSpin());
              }

              return myCommand1.andThen(myCommand2);
            },
            Set.of(drive))
        .withName("Auton_Left_LigerBot");
  }

  private static Command Right_1768() {
    return new DeferredCommand(
            () -> {
              Command myCommand1;
              Command myCommand2;

              myCommand1 =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Right_1768_AB1"),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      IntakeCommands.stopSpin(),
                      ShooterCommands.shoot(false).withTimeout(3.0));

              if (autonOutpost.get()) {
                myCommand2 =
                    Commands.sequence(
                        Commands.parallel(
                            PathUtil.followPath("Right_1768_Outpost"),
                            Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                        Commands.waitSeconds(1)
                            .andThen(ShooterCommands.shoot(false).withTimeout(5.0)),
                        IntakeCommands.stopSpin());
              } else {
                myCommand2 =
                    Commands.sequence(
                        Commands.parallel(
                            PathUtil.followPath("Right_1768_CD1"),
                            Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                        IntakeCommands.stopSpin(),
                        ShooterCommands.shoot(false).withTimeout(5.0));
              }

              return myCommand1.andThen(myCommand2);
            },
            Set.of(drive))
        .withName("Auton_ARight_1768");
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
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      IntakeCommands.stopSpin(),
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
