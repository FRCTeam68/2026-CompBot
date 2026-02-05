package frc.robot;

import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.auton.AutonCommands;
import frc.robot.commands.auton.AutonSequence;
import frc.robot.commands.auton.AutonSequenceCenter;
import frc.robot.commands.auton.AutonSequenceRightTrench;
import frc.robot.util.AutonUtil;
import frc.robot.util.ShiftUtil;
import frc.robot.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // System
  private final System system = System.getInstance();
  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandPS4Controller operatorController = new CommandPS4Controller(1);

  // Alerts
  private final Alert driverControllerDisconnectedAlert =
      new Alert("Driver Xbox controller disconnected.", AlertType.kError);
  private final Alert operatorControllerDisconnectedAlert =
      new Alert("Operator PS4 controller disconnected.", AlertType.kError);
  private final Alert noAutoSelectedAlert =
      new Alert("No autonomous routine selected.", AlertType.kWarning);
  private final Alert startingPoseAlert =
      new Alert(
          "Current robot pose does not match the starting pose for selected auton. Possible causes include the incorrect auton is selected, the camera is not getting a clear view of an april tag, or the robot is in the wrong location.",
          AlertType.kError);

  // Dashboard inputs
  private final LoggedDashboardChooser<AutonSequence> autonChooser;

  private final Trigger hubTransitionWarningTrigger =
      new Trigger(() -> ShiftUtil.hubToActiveWarning(3) || ShiftUtil.hubToInactiveWarning(3));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure dashboard
    SmartDashboard.putData(
        "Move To Starting Pose",
        Commands.runOnce(() -> Commands.none()).andThen(() -> stopSubsystems()));

    // Configure auton chooser
    autonChooser = new LoggedDashboardChooser<>("Auton Chooser");
    autonChooser.addDefaultOption("NONE", null);
    autonChooser.addOption("Middle Depot Auto", new AutonSequenceCenter());
    autonChooser.addOption("Right Trench Auto Climber", new AutonSequenceRightTrench(false));
    autonChooser.addOption("Right Trench Auto Feeder", new AutonSequenceRightTrench(true));
  }

  /** Use this method to define button -> command mappings. */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    system
        .getDrive()
        .setDefaultCommand(
            DriveCommands.joystickDrive(
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

    driverController
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        system
                            .getDrive()
                            .setPose(
                                new Pose2d(
                                    system.getDrive().getPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true));

    driverController.start().onTrue(Commands.runOnce(() -> stopSubsystems()).ignoringDisable(true));
    driverController
        .povUp()
        .onTrue(
            DriveCommands.autopilotDriveToPose(
                () ->
                    new APTarget(
                        AllianceFlipUtil.apply(
                            FieldConstants.Hub.nearFace.transformBy(
                                new Transform2d(2.0, 0.0, Rotation2d.kPi))))));
    driverController
        .povLeft()
        .onTrue(
            DriveCommands.autopilotDriveToPose(
                drive,
                () ->
                    new APTarget(
                            AllianceFlipUtil.apply(
                                new Pose2d(FieldConstants.Hub.nearLeftCorner, new Rotation2d())
                                    .transformBy(new Transform2d(-0.5, 0.0, Rotation2d.kPi))))
                        .withEntryAngle(AllianceFlipUtil.apply(Rotation2d.kZero))));
    driverController
        .povDown()
        .onTrue(
            DriveCommands.autopilotDriveToPose(
                drive,
                () ->
                    new APTarget(
                            AllianceFlipUtil.apply(
                                new Pose2d(FieldConstants.Hub.nearRightCorner, new Rotation2d())
                                    .transformBy(new Transform2d(-0.5, 0.0, Rotation2d.kPi))))
                        .withEntryAngle(AllianceFlipUtil.apply(Rotation2d.kZero))));

    hubTransitionWarningTrigger.onTrue(
        Commands.runOnce(() -> driverController.setRumble(RumbleType.kBothRumble, 1))
            .andThen(Commands.waitSeconds(1))
            .andThen(() -> driverController.setRumble(RumbleType.kBothRumble, 0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutonCommands.autonCommand(autonChooser.get());
  }

  /** Loads autonomous paths from storage. This method can be safely be called periodically. */
  public void loadAutonomousPath() {
    AutonUtil.loadPaths(autonChooser.get() != null ? autonChooser.get().getPathNames() : null);
  }

  /** Stops all subsystems, cancels all scheduled commands, and stops controller rumble. */
  public void stopSubsystems() {
    CommandScheduler.getInstance().cancelAll();
    driverController.setRumble(RumbleType.kBothRumble, 0);
    operatorController.setRumble(RumbleType.kBothRumble, 0);
    system.getDrive().stop();
  }

  /**
   * <b>Alerts always active:</b>
   *
   * <ul>
   *   <li>Controllers disconnected
   * </ul>
   *
   * <b>Alerts only active while in autonomous and disabled:</b>
   *
   * <ul>
   *   <li>No autonomous selected
   *   <li>Current pose does not match autonomous starting pose
   * </ul>
   */
  public void updateAlerts() {
    driverControllerDisconnectedAlert.set(!driverController.isConnected());
    operatorControllerDisconnectedAlert.set(!operatorController.isConnected());

    if (DriverStation.isAutonomous() && DriverStation.isDisabled()) {
      noAutoSelectedAlert.set(autonChooser.get() == null);
      startingPoseAlert.set(
          autonChooser.get() != null
              && (AutonUtil.getStartingPose()
                          .minus(system.getDrive().getPose())
                          .getTranslation()
                          .getNorm()
                      > 0.25
                  || AutonUtil.getStartingPose()
                          .minus(system.getDrive().getPose())
                          .getRotation()
                          .getDegrees()
                      > 20));
    } else {
      noAutoSelectedAlert.set(false);
      startingPoseAlert.set(false);
    }
  }
}
