package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import lombok.Builder;

public final class DriveConstants {
  // Hardware Configuration
  public static final CANBus canbus = new CANBus("rio");
  public static final double trackWidthX = Units.inchesToMeters(22.5); // meters
  public static final double trackWidthY = Units.inchesToMeters(22.5); // meters
  public static final double driveReduction = 5.14;
  public static final double turnReduction = 12.8;
  public static final double maxLinearVelocity = 6.27; // meters/second | max:6.27
  public static final double wheelRadius = Units.inchesToMeters(2); // meters
  private static final double mass = 68; // kilograms
  private static final double moi = 1; // kilogram*meters^2

  // Control Configuration
  // TODO: do we want to limit torque, stator, and or supply current
  // public static final double driveTorqueCurrentLimit = 80; // amps
  // public static final double driveSupplyCurrentLimit = 80; // amps
  // public static final double turnSupplyCurrentLimit = 40; // amps
  public static final double driveCurrentLimitAmps = 80; // amps
  public static final double turnCurrentLimitAmps = 40; // amps
  public static final double maxLinearAcceleration = 22; // meters/second^2
  public static final double maxAngularAcceleration = 22;

  // Derived values (No need to change)
  public static final double odometryFrequency = canbus.isNetworkFD() ? 250.0 : 100.0;
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double maxAngularVelocity = maxLinearVelocity / driveBaseRadius;
  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  // Simple profiled PID configuration
  public static final PIDConstants linearPID = new PIDConstants(5.0, 0.0, 0.0);
  public static final PIDConstants angularPID = new PIDConstants(4.0, 0.0, 0.0);

  // Autopilot configuration
  public static final APProfile apConfigStatic =
      new APProfile(new APConstraints(100.0, 100.0))
          .withErrorXY(Inches.of(3))
          .withErrorTheta(Degrees.of(3))
          .withBeelineRadius(Inches.of(12));

  public static final APProfile apConfigDynamic =
      new APProfile(new APConstraints(100.0, 10))
          .withErrorXY(Inches.of(6))
          .withErrorTheta(Degrees.of(6))
          .withBeelineRadius(Inches.of(12));

  // PathPlanner configuration
  public static final RobotConfig ppConfig =
      new RobotConfig(
          mass,
          moi,
          new com.pathplanner.lib.config.ModuleConfig(
              wheelRadius,
              maxLinearVelocity,
              1,
              DCMotor.getKrakenX60Foc(1).withReduction(driveReduction),
              driveCurrentLimitAmps,
              1),
          moduleTranslations);

  public static final String[] moduleNames = {
    "module 0 (Front Left)",
    "module 1 (Front Right)",
    "module 2 (Back Left)",
    "module 3 (Back Right)"
  };

  public static final ModuleConfig[] moduleConfigs = {
    // Module 0 (Front Left)
    ModuleConfig.builder()
        .driveMotorId(1)
        .turnMotorId(2)
        .encoderId(15)
        .encoderOffset(Rotation2d.fromRotations(-0.1552734375))
        .turnInverted(false)
        .encoderInverted(false)
        .build(),
    // Module 1 (Front Right)
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(4)
        .encoderId(16)
        .encoderOffset(Rotation2d.fromRotations(-0.150634765625).rotateBy(Rotation2d.kPi))
        .turnInverted(false)
        .encoderInverted(false)
        .build(),
    // Module 2 (Back Left)
    ModuleConfig.builder()
        .driveMotorId(5)
        .turnMotorId(6)
        .encoderId(17)
        .encoderOffset(Rotation2d.fromRotations(0.19580078125))
        .turnInverted(false)
        .encoderInverted(false)
        .build(),
    // Module 3 (Back Right)
    ModuleConfig.builder()
        .driveMotorId(7)
        .turnMotorId(8)
        .encoderId(18)
        .encoderOffset(Rotation2d.fromRotations(0.3154296875).rotateBy(Rotation2d.kPi))
        .turnInverted(false)
        .encoderInverted(false)
        .build(),
  };

  public static class GyroConstants {
    public static final int id = 50;
  }

  @Builder
  public record ModuleConfig(
      int driveMotorId,
      int turnMotorId,
      int encoderId,
      Rotation2d encoderOffset,
      boolean turnInverted,
      boolean encoderInverted) {}
}
