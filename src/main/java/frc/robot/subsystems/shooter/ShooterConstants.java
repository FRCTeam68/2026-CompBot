package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

  public static class Target {
    public static final Translation2d hub = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
    public static final Translation2d passLeft =
        new Translation2d(1, FieldConstants.fieldWidth - 1);
    public static final Translation2d passRight = new Translation2d(1, 1);
  }

  public static class StaticShot {
    public static final shotConfig hubArc =
        shotConfig.builder().flywheelVelocity(0.000).hoodAngle(0.000).turretAngle(0.000).build();
    public static final shotConfig neutralZone =
        shotConfig.builder().flywheelVelocity(0.000).hoodAngle(0.000).turretAngle(0.000).build();
    public static final shotConfig oppAllianceZone =
        shotConfig.builder().flywheelVelocity(0.000).hoodAngle(0.000).turretAngle(0.000).build();
  }

  public static class DynamicShot {
    public static final double adjustMultiplier = 1.0;

    private static final InterpConfig[] hubConfigLow = {
      InterpConfig.builder().distance(1.18).flywheel(45).hood(79.37).flightTime(1).build(),
      InterpConfig.builder().distance(1.68).flywheel(40).hood(70).flightTime(1).build(),
      InterpConfig.builder().distance(2.24).flywheel(42).hood(63).flightTime(1).build(),
      InterpConfig.builder().distance(3.46).flywheel(46).hood(57).flightTime(1).build(),
      InterpConfig.builder().distance(5.1).flywheel(52).hood(45).flightTime(1).build()
    };

    private static final InterpConfig[] hubConfigHigh = {
      InterpConfig.builder().distance(1.18).flywheel(45).hood(79.37).flightTime(1).build(),
      InterpConfig.builder().distance(1.68).flywheel(40).hood(70).flightTime(1).build(),
      InterpConfig.builder().distance(2.24).flywheel(42).hood(63).flightTime(1).build(),
      InterpConfig.builder().distance(3.46).flywheel(46).hood(57).flightTime(1).build(),
      InterpConfig.builder().distance(5.1).flywheel(52).hood(45).flightTime(1).build()
    };

    private static final InterpConfig[] passConfigLow = {
      InterpConfig.builder().distance(1.18).flywheel(45).hood(79.37).flightTime(1).build(),
      InterpConfig.builder().distance(1.68).flywheel(40).hood(70).flightTime(1).build(),
      InterpConfig.builder().distance(2.24).flywheel(42).hood(63).flightTime(1).build(),
      InterpConfig.builder().distance(3.46).flywheel(46).hood(57).flightTime(1).build(),
      InterpConfig.builder().distance(5.1).flywheel(52).hood(45).flightTime(1).build()
    };

    private static final InterpConfig[] passConfigHigh = {
      InterpConfig.builder().distance(1.18).flywheel(45).hood(79.37).flightTime(1).build(),
      InterpConfig.builder().distance(1.68).flywheel(40).hood(70).flightTime(1).build(),
      InterpConfig.builder().distance(2.24).flywheel(42).hood(63).flightTime(1).build(),
      InterpConfig.builder().distance(3.46).flywheel(46).hood(57).flightTime(1).build(),
      InterpConfig.builder().distance(5.1).flywheel(52).hood(45).flightTime(1).build()
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
