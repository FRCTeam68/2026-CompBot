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
import frc.robot.commands.auton.Auton;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ShiftUtil;
import frc.robot.util.geometry.AllianceFlipUtil;

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
      new Trigger(() -> ShiftUtil.hubToActiveWarning(3) || ShiftUtil.hubToInactiveWarning(3));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure dashboard
    SmartDashboard.putData(
        "Move To Starting Pose",
        Commands.runOnce(() -> Commands.none()).andThen(() -> stopSubsystems()));

    Auton.initDashboardInputs();
  }

  /** Use this method to define button -> command mappings. */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

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
                () ->
                    new APTarget(
                            AllianceFlipUtil.apply(
                                new Pose2d(FieldConstants.Hub.nearRightCorner, new Rotation2d())
                                    .transformBy(new Transform2d(-0.5, 0.0, Rotation2d.kPi))))
                        .withEntryAngle(AllianceFlipUtil.apply(Rotation2d.kZero))));

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

    driverController.leftTrigger().whileTrue(IntakeCommands.intakeWhile());

    hubTransitionWarningTrigger.onTrue(
        Commands.runOnce(() -> driverController.setRumble(RumbleType.kBothRumble, 1))
            .andThen(Commands.waitSeconds(1))
            .andThen(() -> driverController.setRumble(RumbleType.kBothRumble, 0))
            .withName("HubTransitionWarning"));

    driverController.b().whileTrue(IntakeCommands.outtake());

    driverController.leftBumper().onTrue(IntakeCommands.retract());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Auton.command();
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
  }

  /** Save Limelight 4 rewind to disc. This is only functional on the Limelight 4. */
  public void saveLimelightRewind() {
    vision.saveLimelightRewind();
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
    robotSystem.visualization();
    driverControllerDisconnectedAlert.set(!driverController.isConnected());
    operatorControllerDisconnectedAlert.set(!operatorController.isConnected());
  }
}
