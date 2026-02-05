package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
import lombok.Getter;

public class System {
  private static System instance = null;

  // Subsystems
  @Getter private final Drive drive;
  @Getter private final Vision vision;

  private final ShotVisualizer shotVisualizer;

  public System() {
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
  }

  /**
   * Gets the single instance of the System class.
   *
   * @return The single instance of the System class.
   */
  public static System getInstance() {
    if (instance == null) {
      instance = new System();
    }
    return instance;
  }
}
