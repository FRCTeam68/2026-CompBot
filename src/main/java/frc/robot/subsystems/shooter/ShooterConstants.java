package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.util.CanBusUtil;
import lombok.Builder;

public class ShooterConstants {
  /**
   * The robot relative position of the shooter. It is in line with the axis of rotation for the
   * turret and at the height of the flywheel axis.
   */
  public static final Translation3d shooterPosition =
      new Translation3d(-0.160018476, 0.1335875408, 0.4431027206);

  public static final CANBus canBus = CanBusUtil.getCanivoreBus();

  /**
   * The amount of time it takes fuel to get from the top of the hub chute to the counting sensor.
   */
  public static final double hubFilterTime = 1.5;

  public static class TrenchZone {
    // The maximum time for the hood to lower to underTrenchMinimum
    public static final double hoodLowerTime = 0.5;

    // Default box size. xMin should be set big enough to allow ample time for the hood to go down
    // from 0 velocity.
    public static final double halfXSize = Units.inchesToMeters(47) / 2.0;
    public static final double ySize =
        FieldConstants.LinesHorizontal.rightTrenchOpenStart + Units.inchesToMeters(12);
  }

  public static class TowerZone {
    public static final double xSize = Units.inchesToMeters(43.5);
    public static final double yMin = Units.inchesToMeters(107);
    public static final double yMax = Units.inchesToMeters(180);
    public static final double halfYSize = (yMax - yMin) / 2.0;
  }

  public static class BehindHubZone {
    // Zone created is an equilateral triangle centered on the hub
    // Both hubs use the same zone, but it should be optimized for the near hub.
    public static final double halfBaseWidth = Units.inchesToMeters(60) / 2.0;
    public static final double halfHeight = Units.inchesToMeters(120) / 2.0;
    public static final double slope = halfBaseWidth / (halfHeight * 2.0);
  }

  public static class Target {
    public static final Translation2d hub = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
    public static final Translation2d passRight = new Translation2d(3, 2);
    public static final Translation2d passLeft =
        new Translation2d(passRight.getX(), FieldConstants.fieldWidth - passRight.getY());
  }

  public static class StaticShot {
    public static final shotConfig hubArc =
        shotConfig.builder().flywheelVelocity(55.0).hoodAngle(50.0).turretAngle(0.0).build();
    public static final shotConfig neutralZone =
        shotConfig.builder().flywheelVelocity(66.0).hoodAngle(46).turretAngle(0.0).build();
    public static final shotConfig oppAllianceZone =
        shotConfig.builder().flywheelVelocity(66.0).hoodAngle(46).turretAngle(0.0).build();
  }

  public static class DynamicShot {
    public static final double angularMultiplier = 1.0;
    public static final double linearTowardMultiplier = 1.0;
    public static final double linearAwayMultiplier = 1.0;

    public static final double minHubShotDistance = 0.92;

    private static final InterpConfig[] hubConfigHigh = {
      InterpConfig.builder().distance(0.92).flywheel(50).hood(72).flightTime(1.1).build(),
      InterpConfig.builder().distance(1.91).flywheel(50).hood(65).flightTime(1.017).build(),
      InterpConfig.builder().distance(2.55).flywheel(55).hood(60).flightTime(0.967).build(),
      InterpConfig.builder().distance(2.95).flywheel(55).hood(54.5).flightTime(1.033).build(),
      InterpConfig.builder().distance(3.50).flywheel(58).hood(53).flightTime(1.033).build(),
      InterpConfig.builder().distance(4.03).flywheel(60.5).hood(50).flightTime(1.017).build(),
      InterpConfig.builder().distance(4.56).flywheel(63).hood(50).flightTime(1.15).build(),
      InterpConfig.builder().distance(5.02).flywheel(64.5).hood(50).flightTime(1).build(),
      InterpConfig.builder().distance(5.20).flywheel(67.0).hood(50).flightTime(1).build()
    };

    private static final InterpConfig[] passConfigHigh = {
      InterpConfig.builder().distance(4.77).flywheel(56).hood(46).flightTime(1.3).build(),
      InterpConfig.builder().distance(5.91).flywheel(61).hood(46).flightTime(1.4).build(),
      InterpConfig.builder().distance(6.79).flywheel(66).hood(46).flightTime(1.45).build()
    };

    private static final InterpConfig[] hubConfigLow = {
      InterpConfig.builder().distance(0.92).flywheel(50).hood(72).flightTime(1.1).build(),
      InterpConfig.builder().distance(1.91).flywheel(50).hood(65).flightTime(1.017).build(),
      InterpConfig.builder().distance(2.55).flywheel(55).hood(60).flightTime(0.967).build(),
      InterpConfig.builder().distance(2.95).flywheel(55).hood(54.5).flightTime(1.033).build(),
      InterpConfig.builder().distance(3.50).flywheel(58).hood(53).flightTime(1.033).build(),
      InterpConfig.builder().distance(4.03).flywheel(60.5).hood(50).flightTime(1.017).build(),
      InterpConfig.builder().distance(4.56).flywheel(63).hood(50).flightTime(1.15).build(),
      InterpConfig.builder().distance(5.02).flywheel(64.5).hood(50).flightTime(1).build(),
      InterpConfig.builder().distance(5.20).flywheel(67.0).hood(50).flightTime(1).build()
    };

    private static final InterpConfig[] passConfigLow = {
      InterpConfig.builder().distance(4.77).flywheel(56).hood(46).flightTime(1.3).build(),
      InterpConfig.builder().distance(5.91).flywheel(61).hood(46).flightTime(1.4).build(),
      InterpConfig.builder().distance(6.79).flywheel(66).hood(46).flightTime(1.45).build()
    };

    // Config to use based off lowCeiling
    private static final InterpConfig[] hubConfig =
        Constants.lowCeiling ? hubConfigLow : hubConfigHigh;
    private static final InterpConfig[] passConfig =
        Constants.lowCeiling ? passConfigLow : passConfigHigh;

    // Hub shot interpolation tree map
    public static InterpolatingDoubleTreeMap hubShotHoodElevation =
        new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap hubShotFlywheelVelocity =
        new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap hubShotFlightTime = new InterpolatingDoubleTreeMap();

    // pass shot interpolation tree map
    public static InterpolatingDoubleTreeMap passShotHoodElevation =
        new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap passShotFlywheelVelocity =
        new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap passShotFlightTime = new InterpolatingDoubleTreeMap();

    static {
      for (InterpConfig config : hubConfig) {
        hubShotHoodElevation.put(config.distance, config.hood);
        hubShotFlywheelVelocity.put(config.distance, config.flywheel);
        hubShotFlightTime.put(config.distance, config.flightTime);
      }
      for (InterpConfig config : passConfig) {
        passShotHoodElevation.put(config.distance, config.hood);
        passShotFlywheelVelocity.put(config.distance, config.flywheel);
        passShotFlightTime.put(config.distance, config.flightTime);
      }
    }
  }

  /**
   * Static shot configuration.
   *
   * @param flywheelVelocity The velocity of the flywheel in rotations per second.
   * @param hoodAngle The elevation of the hood in degrees.
   * @param turretAngle The counterclockwise angle of the turret in degrees.
   */
  @Builder
  public record shotConfig(double flywheelVelocity, double hoodAngle, double turretAngle) {}

  /**
   * Interpolation table entry configuration. Names are shortened to allow each entry to remain on a
   * single line after formatting.
   *
   * @param distance The distance to the target in meters.
   * @param flywheel The velocity of the flywheel in rotations per second.
   * @param hood The elevation of the hood in degrees.
   * @param flightTime The flight time of the shot.
   */
  @Builder
  public record InterpConfig(double distance, double flywheel, double hood, double flightTime) {}
}
