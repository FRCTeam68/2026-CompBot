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
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.PathUtil;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Auton {
  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final Drive drive = robotSystem.getDrive();
  private static final Vision vision = robotSystem.getVision();
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

  private static final LoggedNetworkBoolean setStartingPose =
      new LoggedNetworkBoolean("SmartDashboard/Auton/SetStartingPose", false);

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
            return LeftDefault();
        }

      case Center:
        return CenterDefault();

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
            return RightDefault();
        }

      default:
        return Commands.none();
    }
  }

  // -----------------------------------------------------------------------------------------
  private static Command delayShooterStart(double turretPosition) {
    return Commands.sequence(
            // Make sure flywheels don't start
            ShooterCommands.runStatic(
                0, shooter.getHood().getElevation(), shooter.getTurret().getPosition()),
            Commands.waitSeconds(1),
            // Move to the setpoints that will be called when the robot enters the
            // alliance zone again
            // ShooterCommands.runStatic(57, 63, 275),
            ShooterCommands.runStatic(57, 63, turretPosition),
            // enable automatic control
            // The shooter will remain at the previous setpoint until it enters
            // the alliance zone
            ShooterCommands.clearStaticSetpoint())
        .withName("DelayShooterStart");
  }

  private static Command WaitWhileShootThenAgitate() {
    return Commands.sequence(Commands.waitSeconds(3), IntakeCommands.agitate().withTimeout(1))
        .withName("WaitWhileShootThenAgitate");
  }

  private static Command CenterDefault() {
    return new DeferredCommand(
            () -> {
              Command myCommand1;
              Command myCommand2 = Commands.none();

              if (autonDepot.get()) {

                myCommand1 =
                    Commands.sequence(
                        Commands.parallel(
                            PathUtil.followPath("Center Depot"),
                            Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                        Commands.waitSeconds(3).andThen(IntakeCommands.stopSpin()),
                        PathUtil.followPath("Depot Tower"));

                if (autonOutpost.get()) {
                  myCommand2 = Commands.sequence(PathUtil.followPath("Tower Outpost"));
                }
              } else {
                myCommand1 = Commands.sequence(PathUtil.followPath("Center Outpost"));
              }

              return myCommand1.andThen(myCommand2);
            },
            Set.of(drive))
        .withName("Auton_CenterDefault");
  }

  private static Command LeftDefault() {
    return new DeferredCommand(
            () -> {
              Command myCommand1;
              Command myCommand2;
              Command neutralPath1Command;
              Command toDepotCommand;

              neutralPath1Command =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Left Trench A"),
                          delayShooterStart(90),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      PathUtil.followPath("Left Trench B"),
                      IntakeCommands.stopSpin());

              toDepotCommand =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Left Depot1"),
                          ShooterCommands.dontShoot().withTimeout(2),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      PathUtil.followPath("Left Depot2"));

              if (!autonDepot.get()) {
                // run single pass to nuetral zone
                myCommand1 = Commands.sequence(neutralPath1Command, WaitWhileShootThenAgitate());
                myCommand2 = Commands.none();

              } else {
                myCommand1 = neutralPath1Command;
                myCommand2 =
                    Commands.sequence(
                        toDepotCommand, // drive to depot while shooting
                        WaitWhileShootThenAgitate());
              }

              return myCommand1.andThen(myCommand2);
            },
            Set.of(drive))
        .withName("Auton_LeftDefault");
  }

  private static Command Left_LigerBot() {
    return new DeferredCommand(
            () -> {
              Command myCommand1;
              Command myCommand2;
              Command neutralPath1Command;
              Command neutralPath2Command;
              Command toDepotCommand;

              neutralPath1Command =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Left Trench A1"),
                          delayShooterStart(90),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      PathUtil.followPath("Left Trench A2"),
                      PathUtil.followPath("Left Trench B1"),
                      IntakeCommands.stopSpin());

              neutralPath2Command =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Left Trench C1"),
                          delayShooterStart(90),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      PathUtil.followPath("Left Trench D1"),
                      IntakeCommands.stopSpin());

              toDepotCommand =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Left Depot1"),
                          ShooterCommands.dontShoot().withTimeout(2),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      PathUtil.followPath("Left Depot2"));

              if (!autonDepot.get()) {
                // run double pass to nuetral zone
                myCommand1 =
                    Commands.sequence(
                        neutralPath1Command, WaitWhileShootThenAgitate(), neutralPath2Command);
                myCommand2 = Commands.none();

              } else {
                myCommand1 = neutralPath1Command;
                myCommand2 =
                    Commands.sequence(
                        toDepotCommand, // drive to depot while shooting
                        WaitWhileShootThenAgitate());
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
              Command neutralPath1Command;
              Command neutralPath2Command;
              Command toOutpostCommand;

              neutralPath1Command =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Right_1768_AB1"),
                          delayShooterStart(275),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))));

              neutralPath2Command =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Right_1768_CD1"),
                          delayShooterStart(275),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      IntakeCommands.stopSpin());

              toOutpostCommand =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Right_1768_Outpost_1"),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))));
              // -----------------------------------------------------
              if (!autonOutpost.get()) {
                // run double pass to nuetral zone
                myCommand1 =
                    Commands.sequence(
                        neutralPath1Command,
                        WaitWhileShootThenAgitate(),
                        neutralPath2Command,
                        WaitWhileShootThenAgitate());
                myCommand2 = Commands.none();
              } else {
                // run single pass to nuetral zone, then to outpost while shooting, wait at outpost
                // to shoot, then drive back to trench while shooting
                myCommand1 = neutralPath1Command;
                myCommand2 =
                    Commands.sequence(
                        toOutpostCommand,
                        Commands.waitSeconds(2), // for hopper dump of fuels to finish
                        Commands.parallel(
                            PathUtil.followPath("Right_1768_Outpost_2"),
                            WaitWhileShootThenAgitate()));
              }

              return myCommand1.andThen(myCommand2);
            },
            Set.of(drive))
        .withName("Auton_Right_1768");
  }

  private static Command RightDefault() {
    return new DeferredCommand(
            () -> {
              Command myCommand1;
              Command myCommand2;
              Command neutralPath1Command;
              Command toOutpostCommand;

              neutralPath1Command =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Right Trench A"),
                          delayShooterStart(90),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      PathUtil.followPath("Right Trench B"),
                      IntakeCommands.stopSpin());

              toOutpostCommand =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Right Outpost"),
                          ShooterCommands.dontShoot().withTimeout(2),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      PathUtil.followPath("Outpost Tower"));

              if (!autonOutpost.get()) {
                // run single pass to nuetral zone
                myCommand1 = Commands.sequence(neutralPath1Command, WaitWhileShootThenAgitate());
                myCommand2 = Commands.none();

              } else {
                myCommand1 = neutralPath1Command;
                myCommand2 =
                    Commands.sequence(
                        toOutpostCommand, // drive to outpost while shooting
                        WaitWhileShootThenAgitate());
              }

              return myCommand1.andThen(myCommand2);
            },
            Set.of(drive))
        .withName("Auton_RightDefault");
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

  /**
   * Load starting pose if dashboard toggle is enabled, no cameras are connected, of running in sim
   * mode. If connected to FMS, drive rotation is preserved since we trust the robot was booted
   * straight.
   */
  public static void setStartingPose() {
    if (setStartingPose.get() || !vision.isAnyConnected() || Constants.getMode() == Mode.SIM) {
      if (Constants.getMode() != Mode.SIM) {
        drive.setPose(new Pose2d(getSelectedStartPose().getTranslation(), drive.getRotation()));
      } else {
        drive.setPose(getSelectedStartPose());
      }
    }
  }
}
