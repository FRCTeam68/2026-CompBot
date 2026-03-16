package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotSystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.shotConfig;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterCommands {
  private static final double feederVolts = 12;
  private static final double spindexerVolts = 12;
  private static final double flywheelBumpStep = 0.5;
  private static final double turretBumpStep = 1.0;

  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final Drive drive = robotSystem.getDrive();
  private static final Shooter shooter = robotSystem.getShooter();
  private static final RollerSystem spindexer = robotSystem.getSpindexer();
  private static final RollerSystem feeder = robotSystem.getFeeder();

  @AutoLogOutput(key = "Shooter/ManualShoot")
  private static boolean forceManualShoot = false;

  public static Command shootDefault() {
    return Commands.run(
            () -> {
              if (drive.inAllianceZone() && HubShiftUtil.shouldShoot() && !shooter.staticSetpoint) {
                robotSystem.isShooting = true;

                if (!shooter.holdSetpoint) {
                  if (shooter.atSetpoint() && shooter.inShootableLocation()) {
                    feeder.runVolts(feederVolts);
                    spindexer.runVolts(spindexerVolts);
                  } else {
                    feeder.stop();
                    spindexer.stop();
                  }
                } else {
                  shooter.holdSetpoint = false;
                }
              } else {
                robotSystem.isShooting = false;
                feeder.stop();
                spindexer.stop();
              }
            },
            feeder,
            spindexer)
        .finallyDo(
            () -> {
              feeder.stop();
              spindexer.stop();
              robotSystem.isShooting = false;
            })
        .withName("ShootDefault");
  }

  public static Command shoot(boolean manualMode) {
    return Commands.run(
            () -> {
              if (!shooter.holdSetpoint) {
                if (drive.inAllianceZone() || !shooter.isTargetHub()) {
                  if (manualMode || forceManualShoot) {
                    feeder.runVolts(feederVolts);
                    spindexer.runVolts(spindexerVolts);
                  } else {
                    if (shooter.atSetpoint()
                        && HubShiftUtil.shouldShoot()
                        && shooter.inShootableLocation()) {
                      feeder.runVolts(feederVolts);
                      spindexer.runVolts(spindexerVolts);
                    } else {
                      feeder.stop();
                      spindexer.stop();
                    }
                  }
                }
              } else {
                shooter.holdSetpoint = false;
              }
            },
            feeder,
            spindexer)
        .beforeStarting(
            () -> {
              robotSystem.isShooting = true;
              shooter.shouldTargetPass = true;
            })
        .finallyDo(
            () -> {
              feeder.stop();
              spindexer.stop();
              robotSystem.isShooting = false;
              shooter.shouldTargetPass = false;
            })
        .withName("Shoot");
  }

  public static Command dontShoot() {
    return Commands.idle(feeder, spindexer).withName("DontShoot");
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

  public static Command bumpFlywheel(boolean increaseSpeed) {
    return Commands.runOnce(
            () ->
                shooter.getFlywheel().bumpVelocity +=
                    (increaseSpeed) ? flywheelBumpStep : -flywheelBumpStep)
        .ignoringDisable(true)
        .withName("BumpFlywheel");
  }

  public static Command bumpTurret(boolean increaseAngle) {
    return Commands.runOnce(
            () ->
                shooter.getTurret().bumpAngle += (increaseAngle) ? turretBumpStep : -turretBumpStep)
        .ignoringDisable(true)
        .withName("BumpFlywheel");
  }

  public static Command setHoodForceDown(boolean value) {
    return Commands.runOnce(() -> shooter.getHood().setForceDown(value))
        .ignoringDisable(true)
        .withName("ShooterSetHoodForceDown");
  }

  public static Command setHoldSetpoint(boolean value) {
    return Commands.runOnce(() -> shooter.holdSetpoint = value)
        .ignoringDisable(true)
        .withName("ShooterSetHoldSetpoint");
  }

  /** Toggle the state of alwaysTargetPass. Optionally specify the value to set. */
  public static Command toggleAlwaysTargetPass(boolean... value) {
    return Commands.runOnce(
            () -> robotSystem.alwaysTargetPass.set(!robotSystem.alwaysTargetPass.get()))
        .onlyIf(() -> value.length == 0 || robotSystem.alwaysTargetPass.get() != value[0])
        .ignoringDisable(true)
        .withName("ShooterToggleNoPass");
  }

  /** Toggle the state of forceManualShoot. Optionally specify the value to set. */
  public static Command toggleManualShoot(boolean... value) {
    return Commands.runOnce(() -> forceManualShoot = !forceManualShoot)
        .onlyIf(() -> value.length == 0 || forceManualShoot != value[0])
        .ignoringDisable(true)
        .withName("ShooterToggleManualShoot");
  }

  public static Command clearStaticSetpoint() {
    return Commands.runOnce(() -> shooter.staticSetpoint = false)
        .ignoringDisable(true)
        .withName("ShooterClearStaticSetpoint");
  }
}
