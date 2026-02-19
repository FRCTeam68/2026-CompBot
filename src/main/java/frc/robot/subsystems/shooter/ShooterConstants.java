package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;
import frc.robot.util.CanBusUtil;
import lombok.Builder;

public class ShooterConstants {
  /**
   * The robot relative position of the shooter. It is on the axis of the turret at the height of
   * the flywheel axis.
   */
  public static final Translation3d shooterPosition =
      new Translation3d(-0.160018476, 0.1335875408, 0.4431027206);

  public static final CANBus canBus = CanBusUtil.getCanivoreBus();
  // Interpolation maps
  public static InterpolatingDoubleTreeMap hubShotHoodElevation = new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap hubShotFlywheelVelocity =
      new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap hubShotFlightTime = new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap passShotHoodElevation = new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap passShotFlywheelVelocity =
      new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap passShotFlightTime = new InterpolatingDoubleTreeMap();

  private static final ShooterConfig[] hubConfigLow = {
    ShooterConfig.builder()
        .distance(1)
        .hoodElevation(70)
        .flywheelVelocity(10)
        .flightTime(2)
        .build(),
    ShooterConfig.builder()
        .distance(5)
        .hoodElevation(50)
        .flywheelVelocity(100)
        .flightTime(4)
        .build()
  };

  private static final ShooterConfig[] hubConfigHigh = {
    ShooterConfig.builder()
        .distance(1)
        .hoodElevation(70)
        .flywheelVelocity(10)
        .flightTime(2)
        .build(),
    ShooterConfig.builder()
        .distance(5)
        .hoodElevation(50)
        .flywheelVelocity(100)
        .flightTime(4)
        .build()
  };

  private static final ShooterConfig[] passConfigLow = {
    ShooterConfig.builder()
        .distance(1)
        .hoodElevation(70)
        .flywheelVelocity(10)
        .flightTime(2)
        .build(),
    ShooterConfig.builder()
        .distance(5)
        .hoodElevation(50)
        .flywheelVelocity(100)
        .flightTime(4)
        .build()
  };

  private static final ShooterConfig[] passConfigHigh = {
    ShooterConfig.builder()
        .distance(1)
        .hoodElevation(70)
        .flywheelVelocity(10)
        .flightTime(2)
        .build(),
    ShooterConfig.builder()
        .distance(5)
        .hoodElevation(50)
        .flywheelVelocity(100)
        .flightTime(4)
        .build()
  };

  // Config to use based off lowCeiling
  private static final ShooterConfig[] hubConfig =
      Constants.lowCeiling ? hubConfigLow : hubConfigHigh;
  private static final ShooterConfig[] passConfig =
      Constants.lowCeiling ? passConfigLow : passConfigHigh;

  static {
    for (ShooterConfig config : hubConfig) {
      hubShotHoodElevation.put(config.distance, config.hoodElevation);
      hubShotFlywheelVelocity.put(config.distance, config.flywheelVelocity);
      hubShotFlightTime.put(config.distance, config.flightTime);
    }
    for (ShooterConfig config : passConfig) {
      passShotHoodElevation.put(config.distance, config.hoodElevation);
      passShotFlywheelVelocity.put(config.distance, config.flywheelVelocity);
      passShotFlightTime.put(config.distance, config.flightTime);
    }
  }

  /**
   * @param distance The distance to the target in meters.
   * @param hoodElevation The elevation of the hood in degrees.
   * @param flywheelVelocity The velocity of the flywheel in rotations per second.
   * @param flightTime The flight time of the shot.
   */
  @Builder
  public record ShooterConfig(
      double distance, double hoodElevation, double flywheelVelocity, double flightTime) {}
}
