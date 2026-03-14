package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOReal;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.IntakePivotIO;
import frc.robot.subsystems.intakePivot.IntakePivotIOReal;
import frc.robot.subsystems.intakePivot.IntakePivotIOSim;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.LightsIO;
import frc.robot.subsystems.lights.LightsIOCANdle;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.rollers.RollerSystemIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
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
import frc.robot.util.CanBusUtil;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class RobotSystem {
  private static RobotSystem instance = null;

  public boolean isShooting = false;
  public final LoggedNetworkBoolean doTrenchAlign =
      new LoggedNetworkBoolean("SmartDashboard/Drive/DoTrenchAlign", false);
  public final LoggedNetworkBoolean alwaysTargetPass =
      new LoggedNetworkBoolean("SmartDashboard/Shooter/AlwaysTargetPass", false);

  // Subsystems
  @Getter private final Drive drive;
  @Getter private final Vision vision;
  @Getter private final Lights lights;
  @Getter private final IntakePivot intakePivot;
  @Getter private final RollerSystem intakeSpin;
  @Getter private final Shooter shooter;
  @Getter private final RollerSystem spindexer;
  @Getter private final RollerSystem feeder;

  private final Field2d field = new Field2d();

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
                new VisionIOLimelight(CameraInfo.LL_4), new VisionIOLimelight(CameraInfo.LL_3G));

        flywheel = new Flywheel(new FlywheelIOReal());
        hood = new Hood(new HoodIOReal());
        turret = new Turret(new TurretIOReal());

        intakePivot = new IntakePivot(new IntakePivotIOReal());
        intakeSpin =
            new RollerSystem(
                "intakeSpin",
                new RollerSystemIOTalonFX(
                    22,
                    CanBusUtil.getRioBus(),
                    80,
                    InvertedValue.CounterClockwise_Positive,
                    NeutralModeValue.Coast,
                    24.0 / 18.0));

        spindexer =
            new RollerSystem(
                "spindexer",
                new RollerSystemIOTalonFX(
                    23,
                    CanBusUtil.getCanivoreBus(),
                    80,
                    InvertedValue.Clockwise_Positive,
                    NeutralModeValue.Coast,
                    4.0 * 5.0 * (64.0 / 16.0)));
        feeder =
            new RollerSystem(
                "feeder",
                new RollerSystemIOTalonFX(
                    24,
                    CanBusUtil.getCanivoreBus(),
                    40,
                    InvertedValue.CounterClockwise_Positive,
                    NeutralModeValue.Coast,
                    36.0 / 12.0));

        lights = new Lights(new LightsIOCANdle());
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

        lights = new Lights(new LightsIO() {});

        flywheel = new Flywheel(new FlywheelIOSim());
        hood = new Hood(new HoodIOSim());
        turret = new Turret(new TurretIOSim());

        intakePivot = new IntakePivot(new IntakePivotIOSim() {});
        intakeSpin =
            new RollerSystem(
                "intakeSpin", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.001));

        spindexer =
            new RollerSystem(
                "spindexer",
                new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 3.0 * (64.0 / 16.0), 0.2));
        feeder =
            new RollerSystem(
                "feeder", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 36.0 / 12.0, 0.01));

        lights = new Lights(new LightsIO() {});
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

        lights = new Lights(new LightsIO() {});

        flywheel = new Flywheel(new FlywheelIO() {});
        hood = new Hood(new HoodIO() {});
        turret = new Turret(new TurretIO() {});

        intakePivot = new IntakePivot(new IntakePivotIO() {});
        intakeSpin = new RollerSystem("intakeSpin", (new RollerSystemIO() {}));

        spindexer = new RollerSystem("spindexer", new RollerSystemIO() {});
        feeder = new RollerSystem("feeder", new RollerSystemIO() {});

        lights = new Lights(new LightsIO() {});
    }

    shooter =
        new Shooter(
            flywheel,
            hood,
            turret,
            drive::getPose,
            drive::getFieldVelocity,
            drive::inAllianceZone,
            alwaysTargetPass::get);

    hood.initInTrenchBoxSupplier(shooter::inTrenchBox);
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

  /** Log component poses for the robot visualization. */
  public void visualization() {
    Logger.recordOutput(
        "RobotPose/1_Intake",
        new Pose3d(
            (intakePivot.getPosition() / IntakePivot.getExtended()) * Units.inchesToMeters(12),
            0,
            0,
            Rotation3d.kZero));

    Logger.recordOutput(
        "RobotPose/2_Turret",
        new Pose3d(
            -0.160018476,
            0.1335875408,
            0.0,
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(shooter.getTurret().getPosition()))));

    Logger.recordOutput(
        "RobotPose/3_Hood",
        new Pose3d(
                -0.059053476,
                0.1335875408,
                0.4431027206,
                new Rotation3d(0.0, -Units.degreesToRadians(shooter.getHood().getElevation()), 0.0))
            .rotateAround(
                ShooterConstants.shooterPosition,
                new Rotation3d(
                    0.0, 0.0, Units.degreesToRadians(shooter.getTurret().getPosition()))));

    // Red alliance robot color: #F43636
    // Blue alliance robot color: #3644F4
    field.setRobotPose(drive.getPose());
    SmartDashboard.putData("Field", field);
  }
}
