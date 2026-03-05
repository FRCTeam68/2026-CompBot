package frc.robot;

import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.auton.Auton;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final RobotSystem robotSystem = RobotSystem.getInstance();
  private final Drive drive = robotSystem.getDrive();
  private final Vision vision = robotSystem.getVision();
  private final IntakePivot intakePivot = robotSystem.getIntakePivot();
  private final RollerSystem intakeSpin = robotSystem.getIntakeSpin();
  private final Shooter shooter = robotSystem.getShooter();
  private final RollerSystem feeder = robotSystem.getFeeder();
  private final RollerSystem spindexer = robotSystem.getSpindexer();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandPS4Controller operatorController = new CommandPS4Controller(1);

  // Alerts
  private final Alert driverControllerDisconnectedAlert =
      new Alert("Driver Xbox controller disconnected.", AlertType.kError);
  private final Alert operatorControllerDisconnectedAlert =
      new Alert("Operator PS4 controller disconnected.", AlertType.kError);

  // Triggers
  private final Trigger hubTransitionWarningTrigger =
      new Trigger(() -> HubShiftUtil.hubToActive(3) || HubShiftUtil.hubToInactive(3));

  /** The container for the robot. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure auton dashboard buttons
    Auton.initDashboardInputs();

    // Configure tuning dashboard buttons
    if (Constants.tuningMode) {
      @SuppressWarnings("unused")
      LoggedNetworkString logLabel = new LoggedNetworkString("SmartDashboard/LogLabel", "");
      // Drive
      SmartDashboard.putData(
          "Tuning/DriveLinear_Right",
          DriveCommands.autopilotDriveToPose(
              () ->
                  new APTarget(drive.getPose().plus(new Transform2d(0, -2, Rotation2d.kZero)))
                      .withEntryAngle(Rotation2d.kCW_90deg)));
      SmartDashboard.putData(
          "Tuning/DriveLinear_Left",
          DriveCommands.autopilotDriveToPose(
              () ->
                  new APTarget(drive.getPose().plus(new Transform2d(0, 2, Rotation2d.kZero)))
                      .withEntryAngle(Rotation2d.kCCW_90deg)));
      SmartDashboard.putData(
          "Tuning/DriveAngular_CW",
          DriveCommands.autopilotDriveToPose(
              () ->
                  new APTarget(drive.getPose().plus(new Transform2d(0, 2, Rotation2d.kZero)))
                      .withEntryAngle(Rotation2d.kCCW_90deg)));
      SmartDashboard.putData(
          "Tuning/DriveAngular_CCW",
          DriveCommands.autopilotDriveToPose(
              () ->
                  new APTarget(drive.getPose().plus(new Transform2d(0, 2, Rotation2d.kZero)))
                      .withEntryAngle(Rotation2d.kCCW_90deg)));
      SmartDashboard.putData(
          "Tuning/wheelRadiusCharacterization", DriveCommands.wheelRadiusCharacterization());
      // Turret
      SmartDashboard.putData(
          "Tuning/Turret_1", ShooterCommands.runStatic(0, shooter.getHood().getElevation(), 1));
      SmartDashboard.putData(
          "Tuning/Turret_360", ShooterCommands.runStatic(0, shooter.getHood().getElevation(), 360));
      // Hood
      SmartDashboard.putData(
          "Tuning/Hood_Min",
          ShooterCommands.runStatic(0, Hood.getMinimum() + 3, shooter.getTurret().getPosition()));
      SmartDashboard.putData(
          "Tuning/Hood_Max",
          ShooterCommands.runStatic(0, Hood.getMaximum() - 3, shooter.getTurret().getPosition()));
    }
  }

  /** Use this method to define button -> command mappings. */
  private void configureButtonBindings() {
    // Default drive command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Drive
    // driverController
    //     .povUp()
    //     .onTrue(
    //         DriveCommands.autopilotDriveToPose(
    //             () ->
    //                 new APTarget(
    //                     AllianceFlipUtil.apply(
    //                         FieldConstants.Hub.nearFace.transformBy(
    //                             new Transform2d(2.0, 0.0, Rotation2d.kPi))))));

    // driverController
    //     .povLeft()
    //     .onTrue(
    //         DriveCommands.autopilotDriveToPose(
    //             () ->
    //                 new APTarget(
    //                         AllianceFlipUtil.apply(
    //                             new Pose2d(FieldConstants.Hub.nearLeftCorner, new Rotation2d())
    //                                 .transformBy(new Transform2d(-0.5, 0.0, Rotation2d.kPi))))
    //                     .withEntryAngle(AllianceFlipUtil.apply(Rotation2d.kZero))));

    // driverController
    //     .povRight()
    //     .onTrue(
    //         DriveCommands.autopilotDriveToPose(
    //             () ->
    //                 new APTarget(
    //                         AllianceFlipUtil.apply(
    //                             new Pose2d(FieldConstants.Hub.nearRightCorner, new Rotation2d())
    //                                 .transformBy(new Transform2d(-0.5, 0.0, Rotation2d.kPi))))
    //                     .withEntryAngle(AllianceFlipUtil.apply(Rotation2d.kZero))));

    driverController.povDown().whileTrue(DriveCommands.autopilotDriveToHubArc());

    // Intake
    driverController.leftTrigger().whileTrue(IntakeCommands.intakeWhile());

    driverController.b().whileTrue(IntakeCommands.outtake());

    driverController.leftBumper().whileTrue(IntakeCommands.agitate());

    driverController.povDown().onTrue(IntakeCommands.retract());

    // Shooter
    driverController.rightTrigger().whileTrue(ShooterCommands.shootAutomatic());

    operatorController
        .triangle()
        .onTrue(ShooterCommands.runStatic(ShooterConstants.StaticShot.hubArc));

    operatorController
        .square()
        .onTrue(ShooterCommands.runStatic(ShooterConstants.StaticShot.neutralZone));

    operatorController
        .circle()
        .onTrue(ShooterCommands.runStatic(ShooterConstants.StaticShot.oppAllianceZone));

    operatorController.share().onTrue(ShooterCommands.toggleManualShoot());

    operatorController
        .R2()
        .onTrue(ShooterCommands.setHoldSetpoint(true))
        .onFalse(
            ShooterCommands.setHoldSetpoint(false)
                .alongWith(ShooterCommands.clearStaticSetpoint()));

    operatorController.PS().onTrue(ShooterCommands.toggleNoPass());

    // Misc
    driverController
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true)
                .withName("ResetRobotRotation"));

    driverController
        .start()
        .onTrue(
            Commands.runOnce(() -> stopSubsystems())
                .ignoringDisable(true)
                .withName("StopSubsystems"));

    hubTransitionWarningTrigger.onTrue(
        Commands.runOnce(() -> driverController.setRumble(RumbleType.kBothRumble, 1))
            .andThen(Commands.waitSeconds(1))
            .andThen(() -> driverController.setRumble(RumbleType.kBothRumble, 0))
            .withName("HubTransitionWarning"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Auton.SelectedCommand();
  }

  /** Stops all subsystems, cancels all scheduled commands, and stops controller rumble. */
  public void stopSubsystems() {
    CommandScheduler.getInstance().cancelAll();
    driverController.setRumble(RumbleType.kBothRumble, 0);
    operatorController.setRumble(RumbleType.kBothRumble, 0);
    drive.stop();
    intakePivot.stop();
    intakeSpin.stop();
    shooter.stop();
    feeder.stop();
    spindexer.stop();
  }

  /** Save Limelight 4 rewind to disc. This is only functional on the Limelight 4. */
  public void saveLimelightRewind() {
    vision.saveLimelightRewind();
  }

  /** Log component poses for the robot visualization. */
  public void visualizeRobot() {
    robotSystem.visualization();
  }

  /**
   * Update alerts.
   *
   * <p><b>Alerts always active:</b>
   *
   * <ul>
   *   <li>Controllers disconnected
   * </ul>
   */
  public void updateAlerts() {
    driverControllerDisconnectedAlert.set(!driverController.isConnected());
    operatorControllerDisconnectedAlert.set(!operatorController.isConnected());
  }
}
