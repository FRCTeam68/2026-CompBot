package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.auton.AutonCommands;
import frc.robot.commands.auton.AutonSequence;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOReal;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutonUtil;
import frc.robot.util.FollowPathUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;

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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL -> {
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
      }
      case SIM -> {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                drive::getFieldVelocity,
                new VisionIOSim());
      }
      case REPLAY -> {
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
      }
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
                                AllianceFlipUtil.apply(new Rotation2d()))))
                .ignoringDisable(true));

    driverController.start().onTrue(Commands.runOnce(() -> stopSubsystems()).ignoringDisable(true));
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

  /**
   * Throttle the number of processed frames. This is used to reduce the tempature of the camera.
   * Outputs are not zeroed during skipped frames.
   *
   * <p>This is only applied to the Limelight 4.
   */
  public void setCameraThrottle(boolean throttleCamera) {
    vision.setThrottle(throttleCamera);
  }

  /** Stops all subsystems and cancels all scheduled commands. */
  public void stopSubsystems() {
    CommandScheduler.getInstance().cancelAll();
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
              && (FollowPathUtil.getStartingPose().minus(drive.getPose()).getTranslation().getNorm()
                      > 0.25
                  || FollowPathUtil.getStartingPose()
                          .minus(drive.getPose())
                          .getRotation()
                          .getDegrees()
                      > 20));
    } else {
      noAutoSelectedAlert.set(false);
      startingPoseAlert.set(false);
    }
  }
}
