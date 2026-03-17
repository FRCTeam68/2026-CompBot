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
    TrenchDoubleSweep,
    TrenchSingleSweepPlus,
    BumpSingleSweep,
    Left_Tune_PP_3M_slow,
    Left_Tune_PP_3M_fast,
    Right_Tune_PP_3M_slow
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
    autonSpecial.addOption("TrenchDoubleSweep", Auton.Special.TrenchDoubleSweep);
    autonSpecial.addOption("TrenchSingleSweepPlus", Auton.Special.TrenchSingleSweepPlus);
    if (Constants.tuningMode) {
      autonSpecial.addOption("BumpSingleSweep", Auton.Special.BumpSingleSweep);
      autonSpecial.addOption("Left_Tune_PP_3M_fast", Auton.Special.Left_Tune_PP_3M_fast);
      autonSpecial.addOption("Left_Tune_PP_3M_slow", Auton.Special.Left_Tune_PP_3M_slow);
      autonSpecial.addOption("Right_Tune_PP_3M_slow", Auton.Special.Right_Tune_PP_3M_slow);
    }
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

          case TrenchSingleSweepPlus:
            return Trench(1);
          case TrenchDoubleSweep:
            return Trench(2);
          case BumpSingleSweep:
            return Bump();
          case None:
          default:
            return Commands.none();
        }

      case Center:
        return CenterDefault();

      case Right:
        switch (autonSpecial.get()) {
          case Right_Tune_PP_3M_slow:
            return PathUtil.followPath("Right_Tune_PP_3M_Forward_slow")
                .andThen(Commands.waitSeconds(3))
                .andThen(PathUtil.followPath("Right_Tune_PP_3M_Back_slow"));
          case TrenchSingleSweepPlus:
            return Trench(1);
          case TrenchDoubleSweep:
            return Trench(2);
          case BumpSingleSweep:
            return Bump();
          case None:
          default:
            return Commands.none();
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

  // -----------------------------------------------------------------------------------------
  private static Command CenterDefault() {
    return new DeferredCommand(
            () -> {
              Command myCommand1;

              myCommand1 =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Center_toOutpost"),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      IntakeCommands.stopSpin(),
                      Commands.waitSeconds(3), // wait for fuel to be dumped into hopper
                      PathUtil.followPath("Outpost_toDepot"),
                      IntakeCommands.intakeStatic(false),
                      PathUtil.followPath("Depot_Intake"),
                      Commands.waitSeconds(1), // wait to shoot fuels intaken from depot
                      WaitWhileShootThenAgitate());

              return myCommand1;
            },
            Set.of(drive))
        .withName("Auton_CenterDefault");
  }

  private static Command Trench(Integer numSweeps) {
    return new DeferredCommand(
            () -> {
              boolean left;
              boolean mirror;
              Command myCommand1;
              Command myCommand2;
              Command neutralPath1Command;
              Command neutralPath2Command;

              left = autonStartingPose.get() == StartingPose.Left;
              mirror = left;
              myCommand1 = Commands.none();
              myCommand2 = Commands.none();

              // these 2 paths will be mirrored
              neutralPath1Command =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Right_Trench_Sweep1", mirror),
                          delayShooterStart(mirror ? 90 : 270),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))));

              neutralPath2Command =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Right_Trench_Sweep2", mirror),
                          delayShooterStart(mirror ? 90 : 270),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      IntakeCommands.stopSpin());

              // -----------------------------------------------------
              if (numSweeps == 2) {
                // run double pass to nuetral zone
                myCommand1 =
                    Commands.sequence(
                        neutralPath1Command,
                        WaitWhileShootThenAgitate(),
                        neutralPath2Command,
                        WaitWhileShootThenAgitate());
                myCommand2 = Commands.none();
              } else {
                // -----------------------------------------------------
                // run single pass to nuetral zone,
                myCommand1 = neutralPath1Command;

                // ----  now the 'plus' part of the single sweep plus
                if (!left) {
                  // right side so run to outpost
                  // ..while shooting,
                  // ....wait at outpost to shoot some,
                  // ......drive back to trench while shooting to be ready to go to neutral zone
                  myCommand2 =
                      Commands.sequence(
                          Commands.parallel(
                              PathUtil.followPath("Right_Trench_toOutpost"),
                              Commands.waitSeconds(0.5)
                                  .andThen(IntakeCommands.intakeStatic(false))),
                          IntakeCommands.stopSpin(),
                          Commands.waitSeconds(3), // for hopper dump of fuels to finish
                          Commands.parallel(
                              PathUtil.followPath("Right_Trench_fromOutpost"),
                              WaitWhileShootThenAgitate()));
                } else if (left) {
                  // left side so run to depot
                  // ..then go to depot while NOT shooting,
                  // ....wait at depot to shoot some,
                  // ......drive back to trench while shooting to be ready to go to neutral zone
                  myCommand2 =
                      Commands.sequence(
                          Commands.parallel(
                              PathUtil.followPath("Left_Trench_toDepot"),
                              ShooterCommands.dontShoot().withTimeout(2),
                              Commands.waitSeconds(0.5)
                                  .andThen(IntakeCommands.intakeStatic(false))),
                          PathUtil.followPath("Depot_Intake"),
                          WaitWhileShootThenAgitate());
                } else {
                  // run double pass to nuetral zone
                  myCommand2 = Commands.none();
                }
              }

              return myCommand1.andThen(myCommand2);
            },
            Set.of(drive))
        .withName("Auton_Trench_" + numSweeps + "_Sweeps");
  }

  private static Command Bump() {
    return new DeferredCommand(
            () -> {
              boolean left;
              boolean mirror;
              Command myCommand1;

              Command neutralPath1Command;

              left = autonStartingPose.get() == StartingPose.Left;
              mirror = left;

              // these 2 paths will be mirrored
              neutralPath1Command =
                  Commands.sequence(
                      Commands.parallel(
                          PathUtil.followPath("Right_Bump_Slice1A", mirror),
                          delayShooterStart(mirror ? 90 : 270),
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeStatic(false))),
                      IntakeCommands.stopSpin(),
                      PathUtil.followPath("Right_Bump_Slice1B", mirror));

              // -----------------------------------------------------
              // run double pass to nuetral zone
              myCommand1 = Commands.sequence(neutralPath1Command, WaitWhileShootThenAgitate());

              return myCommand1;
            },
            Set.of(drive))
        .withName("Auton_Bump");
  }

  public static Pose2d getSelectedStartPose() {
    Pose2d startPose = AllianceFlipUtil.apply(Pose2d.kZero);

    if (autonStartingPose.get() != null) {

      switch (autonStartingPose.get()) {
        case Left:
          if (autonSpecial.get() == Auton.Special.BumpSingleSweep) {
            startPose =
                AllianceFlipUtil.apply(PathUtil.getStartingPose("Right_Bump_Slice1A", true));
          } else {
            startPose =
                AllianceFlipUtil.apply(PathUtil.getStartingPose("Right_Trench_Sweep1", true));
          }
          break;
        case Center:
          startPose = AllianceFlipUtil.apply(PathUtil.getStartingPose("Center_toOutpost"));
          break;
        case Right:
          if (autonSpecial.get() == Auton.Special.BumpSingleSweep) {
            startPose = AllianceFlipUtil.apply(PathUtil.getStartingPose("Right_Bump_Slice1A"));
          } else {
            startPose = AllianceFlipUtil.apply(PathUtil.getStartingPose("Right_Trench_Sweep1"));
          }
          break;
        default:
          // do nothing, will return default pose of (0, 0)
          break;
      }
    }

    return startPose;
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
