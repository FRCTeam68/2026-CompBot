package frc.robot;

import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOReal;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.shooter.ShotVisualizer;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.AutonUtil;
import frc.robot.util.ShiftUtil;
import frc.robot.util.geometry.AllianceFlipUtil;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  @Getter private final Vision vision;

  private final ShotVisualizer shotVisualizer;

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
    switch (Constants.getMode()) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOReal(DriveConstants.moduleConfigs[0]),
                new ModuleIOReal(DriveConstants.moduleConfigs[1]),
                new ModuleIOReal(DriveConstants.moduleConfigs[2]),
                new ModuleIOReal(DriveConstants.moduleConfigs[3]));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                drive::getFieldVelocity,
                new VisionIOLimelight(CameraInfo.LL_4));

        shotVisualizer = null;
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision = new Vision(drive::addVisionMeasurement, drive::getPose, drive::getFieldVelocity);

        shotVisualizer =
            new ShotVisualizer(
                drive::getPose,
                () -> 2000.0 / 60.0,
                () -> 70.0,
                () -> new Rotation2d(Units.degreesToRadians(25.0)),
                () -> 1.0);
        break;

      case REPLAY:
      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                drive::getFieldVelocity,
                new VisionIO() {});

        shotVisualizer =
            new ShotVisualizer(
                drive::getPose,
                () -> 3000.0 / 60.0,
                () -> 45.0,
                () -> new Rotation2d(Units.degreesToRadians(10.0)),
                () -> 1.0);
    }

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
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true));

    driverController.start().onTrue(Commands.runOnce(() -> stopSubsystems()).ignoringDisable(true));
    driverController
        .povUp()
        .onTrue(
            DriveCommands.autopilotDriveToPose(
                drive,
                () ->
                    new APTarget(
                        AllianceFlipUtil.apply(
                            FieldConstants.Hub.nearFace.transformBy(
                                new Transform2d(2.0, 0.0, Rotation2d.kPi))))));

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
    return AutonCommands.autonCommand(drive, autonChooser.get());
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
    drive.stop();
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
              && (AutonUtil.getStartingPose().minus(drive.getPose()).getTranslation().getNorm()
                      > 0.25
                  || AutonUtil.getStartingPose().minus(drive.getPose()).getRotation().getDegrees()
                      > 20));
    } else {
      noAutoSelectedAlert.set(false);
      startingPoseAlert.set(false);
    }
  }
}
