package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotSystem;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.shotConfig;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterCommands {
  private static final double feederRunVolts = 12;
  private static final double spindexerRunVolts = 12;

  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final Shooter shooter = robotSystem.getShooter();
  private static final RollerSystem spindexer = robotSystem.getSpindexer();
  private static final RollerSystem feeder = robotSystem.getFeeder();

  @AutoLogOutput(key = "Shooter/ManualShoot")
  private static boolean forceManualShoot = false;

  public static Command shootAutomatic() {
    return Commands.run(
            () -> {
              if (!shooter.holdSetpoint) {
                if (shooter.atSetpoint() && HubShiftUtil.shouldShoot()) {
                  feeder.runVolts(feederRunVolts);
                  spindexer.runVolts(spindexerRunVolts);
                } else {
                  feeder.stop();
                  spindexer.stop();
                }
              } else {
                shooter.holdSetpoint = false;
              }
            },
            feeder,
            spindexer)
        .beforeStarting(() -> robotSystem.isShooting = true)
        .finallyDo(
            () -> {
              feeder.stop();
              spindexer.stop();
              robotSystem.isShooting = false;
            })
        .withName("ShootAutomatic");
  }

  public static Command shootManual() {
    return Commands.runOnce(
            () -> {
              feeder.runVolts(feederRunVolts);
              spindexer.runVolts(spindexerRunVolts);
            })
        .beforeStarting(() -> robotSystem.isShooting = true)
        .finallyDo(
            () -> {
              feeder.stop();
              spindexer.stop();
              robotSystem.isShooting = false;
            })
        .withName("ShootManual");
  }

  public static Command runStatic(
      double flywheelVelocity, double hoodElevation, double turretPosition) {
    return Commands.runOnce(
            () -> shooter.runStatic(flywheelVelocity, hoodElevation, turretPosition), shooter)
        .withName("ShootStatic");
  }

  public static Command runStatic(shotConfig config) {
    return runStatic(config.flywheelVelocity(), config.hoodAngle(), config.turretAngle());
  }

  public static Command stop() {
    return Commands.sequence(Commands.runOnce(() -> shooter.stop(), shooter))
        .withName("ShooterStop");
  }

  public static Command setHoldSetpoint(boolean value) {
    return Commands.runOnce(() -> shooter.holdSetpoint = value)
        .ignoringDisable(true)
        .withName("ShooterSetHoldSetpoint");
  }

  /** Toggle the state of noPass. Optionally specify the value to set. */
  public static Command toggleNoPass(boolean... value) {
    return Commands.runOnce(() -> shooter.noPass = !shooter.noPass)
        .onlyIf(() -> value.length == 0 || shooter.noPass != value[0])
        .ignoringDisable(true)
        .withName("ShooterToggleNoPass");
  }

  public static Command toggleManualShoot(boolean... value) {
    return Commands.runOnce(() -> forceManualShoot = !forceManualShoot)
        .onlyIf(() -> value.length == 0 || forceManualShoot != value[0])
        .ignoringDisable(true)
        .withName("ToggleManualShoot");
  }
}
