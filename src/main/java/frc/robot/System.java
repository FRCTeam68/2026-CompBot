package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOReal;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivotIO;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutputManager;

public class System {
  private static System instance = null;

  // Subsystems
  @Getter private final Drive drive;
  @Getter private final Vision vision;
  @Getter private final Intake intakePivot;
  @Getter private final RollerSystem intakeSpin;
  @Getter private final Shooter shooter;
  @Getter private final RollerSystem spindexer;
  @Getter private final RollerSystem feeder;

  public System() {
    AutoLogOutputManager.addObject(this);

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
                new VisionIOLimelight(CameraInfo.LL_4),
                new VisionIOLimelight(CameraInfo.LL_3G));
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
                new VisionIO() {},
                new VisionIO() {});
    }

    intakePivot = new Intake(new IntakePivotIO() {});
    intakeSpin = new RollerSystem("null1", new RollerSystemIO() {});
    shooter = new Shooter();
    spindexer = new RollerSystem("null2", new RollerSystemIO() {});
    feeder = new RollerSystem("null3", new RollerSystemIO() {});
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
