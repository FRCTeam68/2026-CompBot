package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOReal;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOReal;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

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
    Flywheel flywheel;
    Hood hood;
    Turret turret;
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
        flywheel = new Flywheel(new FlywheelIOReal());
        hood = new Hood(new HoodIOReal());
        turret = new Turret(new TurretIOReal());

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
        flywheel = new Flywheel(new FlywheelIOSim());
        hood = new Hood(new HoodIOSim());
        turret = new Turret(new TurretIOSim());
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
        flywheel = new Flywheel(new FlywheelIO() {});
        hood = new Hood(new HoodIO() {});
        turret = new Turret(new TurretIO() {});
    }

    intakePivot = new Intake(new IntakePivotIO() {});
    intakeSpin = new RollerSystem("null1", new RollerSystemIO() {});
    shooter = new Shooter(flywheel, hood, turret);
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

  public void visualization() {
    Translation3d shooterPosition = new Translation3d(-0.160018476, 0.1335875408, 0);
    Logger.recordOutput(
        "RobotPose/Hood",
        new Pose3d(
                -0.2609834506,
                0.133624955,
                0.4431027206,
                new Rotation3d(0, -Units.degreesToRadians(shooter.getHood().getPosition()), 0))
            .rotateAround(
                shooterPosition,
                new Rotation3d(
                    0, 0, Units.rotationsToRadians(shooter.getTurret().getPosition()) + Math.PI)));
    Logger.recordOutput(
        "RobotPose/Turret",
        new Pose3d(
            -0.160018476,
            0.1335875408,
            0,
            new Rotation3d(
                0, 0, Units.rotationsToRadians(shooter.getTurret().getPosition()) + Math.PI)));
  }
}
