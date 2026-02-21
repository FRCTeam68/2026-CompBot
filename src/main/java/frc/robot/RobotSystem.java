package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOReal;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.IntakePivotIO;
import frc.robot.subsystems.intakePivot.IntakePivotIOSim;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotSystem {
  private static RobotSystem instance = null;

  // Subsystems
  @Getter private final Drive drive;
  @Getter private final Vision vision;
  @Getter private final IntakePivot intakePivot;
  @Getter private final RollerSystem intakeSpin;
  @Getter private final Shooter shooter;
  @Getter private final RollerSystem spindexer;
  @Getter private final RollerSystem feeder;

  public static class ShooterFunctions {
    @AutoLogOutput(key = "Shooter/Hold")
    public static boolean shooterHold = false;

    @AutoLogOutput(key = "Shooter/Toggle")
    public static boolean manualShootToggle = false;

    @AutoLogOutput(key = "Shooter/NoPass")
    public static boolean noPass = false;
  }

  public RobotSystem() {
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

        // flywheel = new Flywheel(new FlywheelIOReal());
        // hood = new Hood(new HoodIOReal(), drive::getPose);
        // turret = new Turret(new TurretIOReal());
        //         spindexer =
        //     new RollerSystem(
        //         "spindexer",
        //         new RollerSystemIOTalonFX(
        //             23,
        //             CanBusUtil.getCanivoreBus(),
        //             80,
        //             InvertedValue.Clockwise_Positive,
        //             NeutralModeValue.Coast,
        //             4 * 5 * (64 / 16)));
        // feeder =
        //     new RollerSystem(
        //         "feeder",
        //         new RollerSystemIOTalonFX(
        //             24,
        //             CanBusUtil.getCanivoreBus(),
        //             80,
        //             InvertedValue.CounterClockwise_Positive,
        //             NeutralModeValue.Coast,
        //             36 / 12));
        // intakePivot = new IntakePivot(new IntakePivotIOReal());
        //         intakeSpin =
        //             new RollerSystem(
        //                 "intakeSpin", new
        // RollerSystemIOTalonFX(22,CanBusUtil.getRioBus(),80,InvertedValue.CounterClockwise_Positive,NeutralModeValue.Coast,24/18));

        flywheel = new Flywheel(new FlywheelIOSim());
        hood = new Hood(new HoodIOSim(), drive::getPose);
        turret = new Turret(new TurretIOSim());

        intakePivot = new IntakePivot(new IntakePivotIOSim() {});
        intakeSpin =
            new RollerSystem(
                "intakeSpin", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1, 0.74));

        spindexer =
            new RollerSystem(
                "spindexer",
                new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4 * 5 * (64 / 16), 0.1));
        feeder =
            new RollerSystem(
                "feeder", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 36 / 12, 0.1));

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
        hood = new Hood(new HoodIOSim(), drive::getPose);
        turret = new Turret(new TurretIOSim());

        intakePivot = new IntakePivot(new IntakePivotIOSim() {});
        intakeSpin =
            new RollerSystem(
                "intakeSpin", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1, 0.74));

        spindexer =
            new RollerSystem(
                "spindexer",
                new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4 * 5 * (64 / 16), 0.1));
        feeder =
            new RollerSystem(
                "feeder", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 36 / 12, 0.1));
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
        hood = new Hood(new HoodIO() {}, drive::getPose);
        turret = new Turret(new TurretIO() {});

        intakePivot = new IntakePivot(new IntakePivotIO() {});
        intakeSpin = new RollerSystem("intakeSpin", (new RollerSystemIO() {}));

        spindexer = new RollerSystem("spindexer", new RollerSystemIO() {});
        feeder = new RollerSystem("feeder", new RollerSystemIO() {});
    }

    shooter = new Shooter(flywheel, hood, turret, drive::getPose, drive::getFieldVelocity);
  }

  /**
   * Gets the single instance of the System class.
   *
   * @return The single instance of the System class.
   */
  public static RobotSystem getInstance() {
    if (instance == null) {
      instance = new RobotSystem();
    }
    return instance;
  }

  public void visualization() {
    Logger.recordOutput(
        "RobotPose/Intake",
        new Pose3d(
            intakePivot.getPosition() / IntakePivot.getExtended() * Units.inchesToMeters(12),
            0,
            0,
            Rotation3d.kZero));

    Logger.recordOutput(
        "RobotPose/Turret",
        new Pose3d(
            -0.160018476,
            0.1335875408,
            0,
            new Rotation3d(
                0, 0, Units.degreesToRadians(shooter.getTurret().getPosition()) + Math.PI)));

    Logger.recordOutput(
        "RobotPose/Hood",
        new Pose3d(
                -0.2609834506,
                0.133624955,
                0.4431027206,
                new Rotation3d(0, Units.degreesToRadians(shooter.getHood().getElevation()), 0))
            .rotateAround(
                ShooterConstants.shooterPosition,
                new Rotation3d(
                    0, 0, Units.degreesToRadians(shooter.getTurret().getPosition()) + Math.PI)));
  }
}
