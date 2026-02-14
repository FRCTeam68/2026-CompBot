package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Builder;

public class ShooterConstants {
  public static final Translation3d shooterPosition =
      new Translation3d(-0.160018476, 0.1335875408, 0);
  public static final ShooterConfig[] hubConfigLow = {
    ShooterConfig.builder()
        .distance(0.0)
        .hoodElevation(0.0)
        .flywheelVelocity(0.0)
        .shootTime(0.0)
        .build()
  };
  public static final ShooterConfig[] hubConfigHigh = {
    ShooterConfig.builder()
        .distance(0.0)
        .hoodElevation(0.0)
        .flywheelVelocity(0.0)
        .shootTime(0.0)
        .build()
  };
  public static final ShooterConfig[] passConfigLow = {
    ShooterConfig.builder()
        .distance(0.0)
        .hoodElevation(0.0)
        .flywheelVelocity(0.0)
        .shootTime(0.0)
        .build()
  };
  public static final ShooterConfig[] passConfigHigh = {
    ShooterConfig.builder()
        .distance(0.0)
        .hoodElevation(0.0)
        .flywheelVelocity(0.0)
        .shootTime(0.0)
        .build()
  };
  public static InterpolatingDoubleTreeMap[] hubShotTable = new InterpolatingDoubleTreeMap[3];
  public static InterpolatingDoubleTreeMap[] passShotTable = new InterpolatingDoubleTreeMap[3];

  static {
    for (ShooterConfig config : hubConfigLow) {
      hubShotTable[0].put(config.distance, config.hoodElevation);
      hubShotTable[1].put(config.distance, config.flywheelVelocity);
      hubShotTable[2].put(config.distance, config.shootTime);
    }
    for (ShooterConfig config : passConfigLow) {
      passShotTable[0].put(config.distance, config.hoodElevation);
      passShotTable[1].put(config.distance, config.flywheelVelocity);
      passShotTable[2].put(config.distance, config.shootTime);
    }
  }

  @Builder
  public record ShooterConfig(
      double distance, double hoodElevation, double flywheelVelocity, double shootTime) {}
}
