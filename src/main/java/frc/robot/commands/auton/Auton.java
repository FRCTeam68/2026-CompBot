package frc.robot.commands.auton;

import com.therekrab.autopilot.APTarget;
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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakeSpin.IntakeSpin;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.Color;
import frc.robot.subsystems.lights.Lights.LEDSegment;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LoggedTracerStatic;
import frc.robot.util.PathUtil;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Auton {
  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final Drive drive = robotSystem.getDrive();
  private static final Lights lights = robotSystem.getLights();
  private static final Vision vision = robotSystem.getVision();
  private static final IntakePivot intakePivot = robotSystem.getIntakePivot();
  private static final IntakeSpin intakeSpin = robotSystem.getIntakeSpin();
  private static final Shooter shooter = robotSystem.getShooter();
  private static final RollerSystem spindexer = robotSystem.getSpindexer();
  private static final RollerSystem feeder = robotSystem.getFeeder();

  // Dashboard inputs
  private static final LoggedDashboardChooser<Auton.StartingPose> autonStartingPose =
      new LoggedDashboardChooser<>("Auton/StartingPose");

  private static final LoggedDashboardChooser<Auton.Special> autonSpecial =
      new LoggedDashboardChooser<>("Auton/Special");

  private static final LoggedNetworkBoolean autonDepot =
      new LoggedNetworkBoolean("SmartDashboard/Auton/Depot", false);

  private static final LoggedNetworkBoolean autonOutpost =
      new LoggedNetworkBoolean("SmartDashboard/Auton/Outpost", false);

  private static final LoggedNetworkBoolean safe =
      new LoggedNetworkBoolean("SmartDashboard/Auton/Safe", true);

  private static final LoggedNetworkBoolean bump =
      new LoggedNetworkBoolean("SmartDashboard/Auton/Bump", false);

  private static final LoggedNetworkBoolean setStartingPose =
      new LoggedNetworkBoolean("SmartDashboard/Auton/SetStartingPose", false);

  private static final LoggedNetworkNumber startDelay =
      new LoggedNetworkNumber("Auton/FirstSweepDelay", 4.0);

  public static enum StartingPose {
    Left,
    Right,
    Center
  }

  public static enum Special {
    None,
    // BumpSingleSweep,
    // Left_Tune_PP_3M_slow,
    // Left_Tune_PP_3M_fast,
    // Right_Tune_PP_3M_slow,
    ThirdRobotPark,
    ThirdRobotBump
  }

  private static final Alert noAutoSelectedAlert =
      new Alert("No autonomous routine selected.", AlertType.kError);
  private static final Alert startingPoseAlert =
      new Alert(
          "Current robot pose does not match the starting pose for selected auton. Possible causes include the incorrect auton is selected, the camera is not getting a clear view of an april tag, or the robot is in the wrong location.",
          AlertType.kError);
  private static final Alert hoodPoseAlert =
      new Alert(
          "The hood is not in the correct starting position. Ensure hood is at the maximum elevation and then click \"Zero Hood\" in the tuning tab.",
          AlertType.kError);
  private static final Alert intakePoseAlert =
      new Alert(
          "The intake is not in the correct starting position. Ensure intake is fully retracted and then click \"Zero\" in the Intake Pivot list of the tuning tab.",
          AlertType.kError);

  private static final LEDSegment startingPoseLEDSegment = new LEDSegment(0, 0, 0);
  private static final LEDSegment componentPoseLEDSegment = new LEDSegment(4, 4, 0);

  public static void initDashboardInputs() {
    // Configure starting pose
    autonStartingPose.addOption("Left", Auton.StartingPose.Left);
    autonStartingPose.addOption("Center", Auton.StartingPose.Center);
    autonStartingPose.addOption("Right", Auton.StartingPose.Right);

    // Configure special
    autonSpecial.addDefaultOption("None", Auton.Special.None);
    autonSpecial.addDefaultOption("Third Robot Park", Auton.Special.ThirdRobotPark);
    autonSpecial.addDefaultOption("Third Robot Bump", Auton.Special.ThirdRobotBump);
    // if (Constants.tuningMode) {
    //   autonSpecial.addOption("BumpSingleSweep", Auton.Special.BumpSingleSweep);
    //   autonSpecial.addOption("Left_Tune_PP_3M_fast", Auton.Special.Left_Tune_PP_3M_fast);
    //   autonSpecial.addOption("Left_Tune_PP_3M_slow", Auton.Special.Left_Tune_PP_3M_slow);
    //   autonSpecial.addOption("Right_Tune_PP_3M_slow", Auton.Special.Right_Tune_PP_3M_slow);
    // }
  }

  public static void UpdateAlerts() {
    if (DriverStation.isAutonomous()) {
      noAutoSelectedAlert.set(autonStartingPose.get() == null);
      startingPoseAlert.set(
          autonStartingPose.get() != null
              && (getSelectedStartPose().minus(drive.getPose()).getTranslation().getNorm() > 0.25
                  || getSelectedStartPose().minus(drive.getPose()).getRotation().getDegrees()
                      > 20));
      hoodPoseAlert.set(Math.abs(shooter.getHood().getElevation() - Hood.getMaximum()) > 2.0);
      intakePoseAlert.set(!intakePivot.isRetracted());
    } else {
      noAutoSelectedAlert.set(false);
      startingPoseAlert.set(false);
      hoodPoseAlert.set(false);
      intakePoseAlert.set(false);
    }

    if (Constants.tuningMode) {
      if (startingPoseAlert.get()) {
        lights.setSolidColor(Color.Dim.RED, startingPoseLEDSegment);
      } else {
        lights.setSolidColor(Color.Dim.GREEN, startingPoseLEDSegment);
      }
      if (hoodPoseAlert.get() || intakePoseAlert.get()) {
        lights.setSolidColor(Color.Dim.RED, componentPoseLEDSegment);
      } else {
        lights.setSolidColor(Color.Dim.GREEN, componentPoseLEDSegment);
      }
    }
  }

  public static Command SelectedCommand() {
    if (autonStartingPose.get() == null) {
      return Commands.none();
    }
    switch (autonStartingPose.get()) {
      case Left:
        switch (autonSpecial.get()) {
            // case Left_Tune_PP_3M_slow:
            //   return PathUtil.followPath("Left_Tune_PP_3M_Forward_slow")
            //       .andThen(Commands.waitSeconds(3))
            //       .andThen(PathUtil.followPath("Left_Tune_PP_3M_Back_slow"));
            // case Left_Tune_PP_3M_fast:
            //   return PathUtil.followPath("Left_Tune_PP_3M_Forward_fast")
            //       .andThen(Commands.waitSeconds(3))
            //       .andThen(PathUtil.followPath("Left_Tune_PP_3M_Back_fast"));
            // case BumpSingleSweep:
            //   return Bump();
          case ThirdRobotBump:
            return thirdRobotBump();
          case ThirdRobotPark:
            return thirdRobotPark();
          case None:
          default:
            if (bump.get()) {
              return Trench_and_Bump();
            } else {
              return Trench();
            }
        }

      case Center:
        return CenterDefault();

      case Right:
        switch (autonSpecial.get()) {
            // case Right_Tune_PP_3M_slow:
            //   return PathUtil.followPath("Right_Tune_PP_3M_Forward_slow")
            //       .andThen(Commands.waitSeconds(3))
            //       .andThen(PathUtil.followPath("Right_Tune_PP_3M_Back_slow"));
            // case BumpSingleSweep:
            //   return Bump();
          case ThirdRobotBump:
            return thirdRobotBump();
          case ThirdRobotPark:
            return thirdRobotPark();
          case None:
          default:
            if (bump.get()) {
              return Trench_and_Bump();
            } else {
              return Trench();
            }
        }

      default:
        return Commands.none();
    }
  }

  // -----------------------------------------------------------------------------------------
  private static Command delayShooterStart(double turretPosition) {
    return Commands.sequence(
            // Ensure flywheels don't start
            ShooterCommands.runStatic(
                0, shooter.getHood().getElevation(), shooter.getTurret().getPosition()),
            Commands.waitSeconds(2),
            // Move to the setpoints for when the robot enters the alliance zone again
            ShooterCommands.runStatic(57, 63, turretPosition),
            // enable automatic control
            // The shooter will remain at the previous setpoint until it enters the alliance zone
            ShooterCommands.clearStaticSetpoint())
        .withName("DelayShooterStart");
  }

  private static Command shootWithAgitation(double shootSeconds, double agitateDelay) {
    return Commands.deadline(
            Commands.waitSeconds(shootSeconds),
            ShooterCommands.shoot(false),
            Commands.waitSeconds(agitateDelay).andThen(IntakeCommands.agitate()))
        .finallyDo(() -> IntakeCommands.deploy(0).andThen(IntakeCommands.intakeStatic(true)))
        .withName("Auton_ShootWithAgitation");
  }

  // -----------------------------------------------------------------------------------------
  private static Command CenterDefault() {
    return new DeferredCommand(
            () -> {
              Command myCommand1;

              myCommand1 =
                  Commands.sequence(
                      Commands.waitSeconds(startDelay.get()),
                      Commands.parallel(
                          PathUtil.followPath("Center_to_Depot1"),
                          ShooterCommands.runStatic(
                              0,
                              shooter.getHood().getElevation(),
                              shooter.getTurret().getPosition()),
                          Commands.waitSeconds(1)
                              .andThen(IntakeCommands.deploy(2))
                              .andThen(Commands.waitSeconds(0.2))
                              .andThen(IntakeCommands.deploy(2))
                              .andThen(IntakeCommands.intakeStatic(false))),
                      PathUtil.followPath("Depot1_to_Depot2"),
                      PathUtil.followPath("Depot2_to_Tower"),
                      ShooterCommands.clearStaticSetpoint(),
                      shootWithAgitation(4.3, 2.1),
                      Commands.parallel(
                          ShooterCommands.runStatic(
                              0,
                              shooter.getHood().getElevation(),
                              shooter.getTurret().getPosition()),
                          PathUtil.followPath("Tower_Sweep_to_Outpost")),
                      ShooterCommands.clearStaticSetpoint(),
                      shootWithAgitation(99, 1.0));
              return myCommand1;
            },
            Set.of(drive, intakePivot, intakeSpin, shooter, spindexer, feeder))
        .withName("Auton_CenterDefault");
  }

  // -----------------------------------------------------------------------------------------
  private static Command Trench() {
    return new DeferredCommand(
            () -> {
              LoggedTracerStatic.record("CommandInit");
              boolean left = autonStartingPose.get() == StartingPose.Left;
              boolean mirror = left;
              Pose2d trenchApproach =
                  AllianceFlipUtil.apply(PathUtil.getStartingPose("Trench_Trench_Slow", mirror));
              Pose2d shot =
                  AllianceFlipUtil.apply(PathUtil.getEndPose("Trench_Trench_Slow", mirror));
              Command myCommand1 = Commands.none();
              Command myCommand2 = Commands.none();

              // these 2 paths will be mirrored
              Command neutralPath1Command =
                  Commands.sequence(
                      Commands.deadline(
                          PathUtil.followPath(
                              safe.get()
                                  ? "Trench_Sweep1_Straight_Safe"
                                  : "Trench_Sweep1_Straight_Agressive_Sidestep",
                              // "Trench_Sweep1_Loop",
                              mirror),
                          delayShooterStart(mirror ? 270 : 90),
                          Commands.waitSeconds(1)
                              .andThen(IntakeCommands.deploy(2))
                              .andThen(Commands.waitSeconds(0.2))
                              .andThen(IntakeCommands.deploy(2))
                              .andThen(IntakeCommands.intakeStatic(false))),
                      Commands.either(
                          DriveCommands.autopilotDriveToPose(
                                  () -> new APTarget(trenchApproach).withoutEntryAngle(), false)
                              .withTimeout(7)
                              .andThen(PathUtil.followPath("Trench_Trench_Slow", mirror)),
                          PathUtil.followPath("Trench_Trench_Fast", mirror),
                          () ->
                              drive.getPose().minus(trenchApproach).getTranslation().getNorm()
                                  > 0.5),
                      DriveCommands.autopilotDriveToPose(
                              () -> new APTarget(shot).withoutEntryAngle(), false)
                          .withTimeout(5)
                          .onlyIf(
                              () -> drive.getPose().minus(shot).getTranslation().getNorm() > 0.5));

              Command neutralPath2Command =
                  Commands.sequence(
                      Commands.deadline(
                          PathUtil.followPath("Trench_Sweep2", mirror),
                          Commands.waitSeconds(0.25).andThen(IntakeCommands.intakeStatic(false))),
                      IntakeCommands.stopSpin(),
                      DriveCommands.autopilotDriveToPose(
                              () -> new APTarget(trenchApproach).withoutEntryAngle(), false)
                          .withTimeout(7)
                          .onlyIf(
                              () ->
                                  drive.getPose().minus(trenchApproach).getTranslation().getNorm()
                                      > 0.5),
                      PathUtil.followPath("Trench_Trench_Slow", mirror),
                      DriveCommands.autopilotDriveToPose(
                              () -> new APTarget(shot).withoutEntryAngle(), false)
                          .withTimeout(5)
                          .onlyIf(
                              () -> drive.getPose().minus(shot).getTranslation().getNorm() > 0.5));

              myCommand1 = Commands.sequence(neutralPath1Command, shootWithAgitation(6, 2.5));
              // -----------------------------------------------------
              if ((!left && !autonOutpost.get()) || (left && !autonDepot.get())) {
                // run double pass to nuetral zone
                myCommand2 = Commands.sequence(neutralPath2Command, shootWithAgitation(99, 1.0));
              } else {
                // ----  now the 'plus' part of the single sweep plus
                if (!left) {
                  // right side so run to outpost
                  // ..while shooting,
                  // ....wait at outpost to shoot some,
                  // ......drive back to trench while shooting to be ready to go to neutral zone
                  myCommand2 =
                      Commands.parallel(
                          Commands.sequence(
                              Commands.deadline(
                                  PathUtil.followPath("Trench_Outpost_To"),
                                  Commands.waitSeconds(0.5)
                                      .andThen(IntakeCommands.intakeAutomatic())),
                              Commands.waitSeconds(3), // for hopper dump of fuels to finish
                              PathUtil.followPath("Trench_Outpost_From"),
                              shootWithAgitation(99, 3)));
                } else {
                  // left side so run to depot
                  // ..then go to depot while NOT shooting,
                  // ....wait at depot to shoot some,
                  // ......drive back to trench while shooting to be ready to go to neutral zone
                  myCommand2 =
                      Commands.sequence(
                          Commands.deadline(
                              PathUtil.followPath("Trench_Depot_To")
                                  .andThen(Commands.waitSeconds(0.2)),
                              Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeAutomatic())),
                          PathUtil.followPath("Trench_Depot_From"),
                          shootWithAgitation(5, 3));
                }
              }

              return Commands.runOnce(() -> LoggedTracerStatic.record("CommandRun"))
                  .andThen(myCommand1)
                  .andThen(myCommand2);
            },
            Set.of(drive, intakePivot, intakeSpin, shooter, spindexer, feeder))
        .withName("Auton_Trench");
  }

  // -----------------------------------------------------------------------------------------
  private static Command Trench_and_Bump() {
    return new DeferredCommand(
            () -> {
              LoggedTracerStatic.record("CommandInit");
              boolean left = autonStartingPose.get() == StartingPose.Left;
              boolean mirror = left;
              Pose2d bumpApproach =
                  AllianceFlipUtil.apply(PathUtil.getStartingPose("Over_Bump", mirror));
              Pose2d shot = AllianceFlipUtil.apply(PathUtil.getEndPose("Over_Bump", mirror));

              // these 2 paths will be mirrored
              Command neutralPath1Command =
                  Commands.sequence(
                      Commands.deadline(
                          PathUtil.followPath(
                              safe.get()
                                  ? "Trench_Bump_Sweep1_Safe"
                                  : "Trench_Bump_Sweep1_Agressive",
                              mirror),
                          delayShooterStart(mirror ? 270 : 90),
                          Commands.waitSeconds(1)
                              .andThen(IntakeCommands.deploy(2))
                              .andThen(Commands.waitSeconds(0.2))
                              .andThen(IntakeCommands.deploy(2))
                              .andThen(IntakeCommands.intakeStatic(false))),
                      Commands.either(
                          DriveCommands.autopilotDriveToPose(
                                  () -> new APTarget(bumpApproach).withoutEntryAngle(), false)
                              .withTimeout(7)
                              .andThen(PathUtil.followPath("Over_Bump", mirror)),
                          PathUtil.followPath("Over_Bump", mirror),
                          () ->
                              drive.getPose().minus(bumpApproach).getTranslation().getNorm()
                                  > 0.5));
              //         ,
              // DriveCommands.autopilotDriveToPose(
              //         () -> new APTarget(shot).withoutEntryAngle(), false)
              //     .withTimeout(5)
              //     .onlyIf(
              //         () -> drive.getPose().minus(shot).getTranslation().getNorm() > 0.5));

              Command neutralPath2Command =
                  Commands.sequence(
                      Commands.deadline(
                          PathUtil.followPath("Trench_Bump_Sweep2", mirror),
                          ShooterCommands.shoot(false)
                              .withTimeout(1), // shoot for a bit before entering trench
                          IntakeCommands.intakeStatic(true)),
                      DriveCommands.autopilotDriveToPose(
                              () -> new APTarget(bumpApproach).withoutEntryAngle(), false)
                          .withTimeout(7)
                          .onlyIf(
                              () ->
                                  drive.getPose().minus(bumpApproach).getTranslation().getNorm()
                                      > 0.5),
                      PathUtil.followPath("Over_Bump", mirror),
                      DriveCommands.autopilotDriveToPose(
                              () -> new APTarget(shot).withoutEntryAngle(), false)
                          .withTimeout(5)
                          .onlyIf(
                              () -> drive.getPose().minus(shot).getTranslation().getNorm() > 0.5));

              Command driveAndShootCommand =
                  Commands.sequence(
                      Commands.deadline(
                          PathUtil.followPath("Bump_to_Trench", mirror)
                              .andThen(Commands.waitSeconds(1.5)),
                          shootWithAgitation(6, 2.5)));

              Command driveAndShootCommand2 =
                  Commands.sequence(
                      IntakeCommands.intakeStatic(true),
                      Commands.deadline(
                          PathUtil.followPath("Bump_to_Trench", mirror),
                          shootWithAgitation(6, 2.5)));

              return Commands.runOnce(() -> LoggedTracerStatic.record("CommandRun"))
                  .andThen(neutralPath1Command)
                  .andThen(driveAndShootCommand)
                  .andThen(neutralPath2Command)
                  .andThen(driveAndShootCommand2);
            },
            Set.of(drive, intakePivot, intakeSpin, shooter, spindexer, feeder))
        .withName("Auton_Trench_Bump");
  }

  // -----------------------------------------------------------------------------------------
  private static Command thirdRobotPark() {
    return Commands.sequence(
            Commands.deadline(
                Commands.waitSeconds(startDelay.get()),
                IntakeCommands.deploy(0),
                ShooterCommands.shoot(false)),
            PathUtil.followPath("Third_Robot_Park", autonStartingPose.get() == StartingPose.Left),
            Commands.idle(drive, shooter))
        .withName("thirdRobotParkAuton");
  }

  // -----------------------------------------------------------------------------------------
  private static Command thirdRobotBump() {
    AtomicBoolean mirror = new AtomicBoolean(false);
    AtomicReference<APTarget> shot = new AtomicReference<APTarget>(new APTarget(Pose2d.kZero));
    return Commands.sequence(
            Commands.deadline(
                Commands.waitSeconds(startDelay.get()),
                IntakeCommands.deploy(0),
                ShooterCommands.shoot(false)),
            PathUtil.followPath(
                "Third_Robot_Trench_Sweep", autonStartingPose.get() == StartingPose.Left),
            PathUtil.followPath(
                "Third_Robot_Trench_Over_Bump", autonStartingPose.get() == StartingPose.Left),
            DriveCommands.autopilotDriveToPose(() -> shot.get(), false),
            shootWithAgitation(99, 4))
        .beforeStarting(
            () -> {
              mirror.set(autonStartingPose.get() == StartingPose.Left);
              shot.set(
                  new APTarget(
                          AllianceFlipUtil.apply(
                              PathUtil.getEndPose(
                                  "Third_Robot_Trench_To_Shot",
                                  autonStartingPose.get() == StartingPose.Left)))
                      .withoutEntryAngle());
            })
        .withName("thirdRobotBumpAuton");
  }

  // -----------------------------------------------------------------------------------------
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
                          Commands.waitSeconds(0.5).andThen(IntakeCommands.intakeAutomatic())),
                      IntakeCommands.stopSpin(),
                      PathUtil.followPath("Right_Bump_Slice1B", mirror));

              // -----------------------------------------------------
              // run double pass to nuetral zone
              myCommand1 = Commands.sequence(neutralPath1Command, shootWithAgitation(4, 3));

              return myCommand1;
            },
            Set.of(drive, intakePivot, intakeSpin, shooter, spindexer, feeder))
        .withName("Auton_Bump");
  }

  public static Pose2d getSelectedStartPose() {
    Pose2d startPose = AllianceFlipUtil.apply(Pose2d.kZero);

    if (autonStartingPose.get() != null) {

      switch (autonStartingPose.get()) {
        case Left:
          // if (autonSpecial.get() == Auton.Special.BumpSingleSweep) {
          //   startPose =
          //       AllianceFlipUtil.apply(PathUtil.getStartingPose("Right_Bump_Slice1A", true));
          // } else {
          if (autonSpecial.get() == Special.None) {
            startPose =
                AllianceFlipUtil.apply(
                    PathUtil.getStartingPose("Trench_Sweep1_Straight_Safe", true));
          } else {
            startPose =
                AllianceFlipUtil.apply(PathUtil.getStartingPose("Third_Robot_Trench_Sweep", true));
          }
          // }
          break;
        case Center:
          startPose = AllianceFlipUtil.apply(PathUtil.getStartingPose("Center_to_Depot1"));
          break;
        case Right:
          // if (autonSpecial.get() == Auton.Special.BumpSingleSweep) {
          //   startPose = AllianceFlipUtil.apply(PathUtil.getStartingPose("Right_Bump_Slice1A"));
          // } else {
          if (autonSpecial.get() == Special.None) {
            startPose =
                AllianceFlipUtil.apply(PathUtil.getStartingPose("Trench_Sweep1_Straight_Safe"));
          } else {
            startPose =
                AllianceFlipUtil.apply(PathUtil.getStartingPose("Third_Robot_Trench_Sweep"));
          }
          // }
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
