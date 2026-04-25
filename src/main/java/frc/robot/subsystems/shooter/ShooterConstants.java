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
  public static final double hubFilterTime = 0.5;

  public static class TrenchZone {
    // The maximum time for the hood to lower to underTrenchMinimum
    public static final double hoodLowerTime = 0.5;

    // Default box size. xMin should be set big enough to allow ample time for the hood to go down
    // from 0 velocity.
    public static final double halfXSize = Units.inchesToMeters(52) / 2.0;
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
        shotConfig.builder().flywheelVelocity(90.0).hoodAngle(46).turretAngle(0.0).build();
    public static final shotConfig fountain =
        shotConfig.builder().flywheelVelocity(5).hoodAngle(55).turretAngle(280).build();
  }

  public static class DynamicShot {
    public static final double minHubShotDistance = 0.92;

    private static final InterpConfig[] hubConfigHigh = {
      InterpConfig.builder().distance(0.92).flywheel(47).hood(72).flightTime(1.1).build(),
      InterpConfig.builder().distance(1.5).flywheel(47).hood(69).flightTime(1.083).build(),
      InterpConfig.builder().distance(2.0).flywheel(47).hood(65).flightTime(1.017).build(),
      InterpConfig.builder().distance(2.5).flywheel(49).hood(60).flightTime(1.033).build(),
      InterpConfig.builder().distance(2.95).flywheel(51.5).hood(58).flightTime(1.033).build(),
      InterpConfig.builder().distance(3.53).flywheel(55).hood(55).flightTime(1.083).build(),
      InterpConfig.builder().distance(3.60).flywheel(55).hood(58).flightTime(1.12).build(),
      InterpConfig.builder().distance(3.99).flywheel(57).hood(58).flightTime(1.16).build(),
      InterpConfig.builder().distance(4.47).flywheel(58).hood(53).flightTime(1.15).build(),
      InterpConfig.builder().distance(5.32).flywheel(63).hood(51).flightTime(1.28).build()
    };

    private static final InterpConfig[] passConfigHigh = {
      InterpConfig.builder().distance(1.353).flywheel(15).hood(46).flightTime(0.733).build(),
      InterpConfig.builder().distance(1.763).flywheel(25).hood(46).flightTime(0.617).build(),
      InterpConfig.builder().distance(2.683).flywheel(35).hood(46).flightTime(0.883).build(),
      InterpConfig.builder().distance(3.863).flywheel(45).hood(46).flightTime(1.117).build(),
      InterpConfig.builder().distance(4.703).flywheel(50).hood(46).flightTime(1.217).build(),
      InterpConfig.builder().distance(5.563).flywheel(55).hood(46).flightTime(1.333).build(),
      InterpConfig.builder().distance(6.553).flywheel(60).hood(46).flightTime(1.433).build(),
      InterpConfig.builder().distance(7.423).flywheel(65).hood(46).flightTime(1.5).build(),
      InterpConfig.builder().distance(8.413).flywheel(70).hood(46).flightTime(1.6).build(),
      InterpConfig.builder().distance(9.183).flywheel(75).hood(46).flightTime(1.6).build(),
      InterpConfig.builder().distance(9.783).flywheel(80).hood(46).flightTime(1.667).build(),
      InterpConfig.builder().distance(10.283).flywheel(85).hood(46).flightTime(1.683).build(),
      InterpConfig.builder().distance(11.183).flywheel(90).hood(46).flightTime(1.8).build()
    };

    private static final InterpConfig[] hubConfigLow = {
      InterpConfig.builder().distance(0.92).flywheel(47).hood(72).flightTime(1.1).build(),
      InterpConfig.builder().distance(1.5).flywheel(47).hood(69).flightTime(1.083).build(),
      InterpConfig.builder().distance(2.0).flywheel(47).hood(65).flightTime(1.017).build(),
      InterpConfig.builder().distance(2.5).flywheel(49).hood(60).flightTime(1.033).build(),
      InterpConfig.builder().distance(2.95).flywheel(52).hood(58).flightTime(1.033).build(),
      InterpConfig.builder().distance(3.53).flywheel(55).hood(55).flightTime(1.083).build(),
      InterpConfig.builder().distance(3.99).flywheel(55).hood(54).flightTime(1.05).build(),
      InterpConfig.builder().distance(4.57).flywheel(59).hood(46).flightTime(1.017).build(),
      InterpConfig.builder().distance(5.05).flywheel(60).hood(46).flightTime(1.083).build(),
      InterpConfig.builder().distance(5.5).flywheel(62).hood(46).flightTime(1.15).build()
    };

    private static final InterpConfig[] passConfigLow = {
      InterpConfig.builder().distance(1.353).flywheel(15).hood(46).flightTime(0.733).build(),
      InterpConfig.builder().distance(1.763).flywheel(25).hood(46).flightTime(0.617).build(),
      InterpConfig.builder().distance(2.683).flywheel(35).hood(46).flightTime(0.883).build(),
      InterpConfig.builder().distance(3.863).flywheel(45).hood(46).flightTime(1.117).build(),
      InterpConfig.builder().distance(4.703).flywheel(50).hood(46).flightTime(1.217).build(),
      InterpConfig.builder().distance(5.563).flywheel(55).hood(46).flightTime(1.333).build(),
      InterpConfig.builder().distance(6.553).flywheel(60).hood(46).flightTime(1.433).build(),
      InterpConfig.builder().distance(7.423).flywheel(65).hood(46).flightTime(1.5).build()
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
