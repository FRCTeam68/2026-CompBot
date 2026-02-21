package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotSystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.shotConfig;
import frc.robot.util.geometry.AllianceFlipUtil;

public class ShooterCommands {
  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final Drive drive = robotSystem.getDrive();
  private static final Shooter shooter = robotSystem.getShooter();
  private static final RollerSystem spindexer = robotSystem.getSpindexer();
  private static final RollerSystem feeder = robotSystem.getFeeder();

  public static Command shootLoop(boolean manual) {
    return Commands.run(
            () -> {
              if (!manual && !RobotSystem.ShooterFunctions.manualShootToggle) {
                if (!RobotSystem.ShooterFunctions.shooterHold) {
                  if (shooter.atSetpoint()) {
                    feeder.runVolts(12);
                    spindexer.runVolts(12);
                  } else {
                    feeder.stop();
                    spindexer.stop();
                  }
                } else {
                  RobotSystem.ShooterFunctions.shooterHold = false;
                }
              } else {
                feeder.runVolts(12);
                spindexer.runVolts(12);
              }
            },
            feeder,
            spindexer)
        .finallyDo(
            () -> {
              feeder.stop();
              spindexer.stop();
            })
        .withName("ShootLoop");
  }

  public static Command runStatic(
      double flywheelVelocity, double hoodElevation, double turretPosition) {
    return Commands.sequence(
            Commands.runOnce(() -> RobotSystem.ShooterFunctions.shooterHold = true),
            Commands.runOnce(
                () -> shooter.runStatic(flywheelVelocity, hoodElevation, turretPosition), shooter))
        .withName("ShootStatic");
  }

  public static Command runStatic(shotConfig config) {
    return runStatic(config.flywheelVelocity(), config.hoodAngle(), config.turretAngle());
  }

  public static Command runDynamic() {
    return Commands.run(
            () -> {
              if (!RobotSystem.ShooterFunctions.shooterHold) {
                Translation2d target;
                boolean isPass;
                if (shooter.inAllianceZone() || RobotSystem.ShooterFunctions.noPass) {
                  target = AllianceFlipUtil.apply(ShooterConstants.Target.hub);
                  isPass = false;
                } else {
                  if (drive.getPose().getTranslation().getY()
                      < FieldConstants.LinesHorizontal.center) {
                    target =
                        (AllianceFlipUtil.shouldFlip())
                            ? ShooterConstants.Target.passLeft
                            : ShooterConstants.Target.passRight;
                  } else {
                    target =
                        (AllianceFlipUtil.shouldFlip())
                            ? ShooterConstants.Target.passRight
                            : ShooterConstants.Target.passLeft;
                  }
                  target = AllianceFlipUtil.apply(target);
                  isPass = true;
                }

                if (isPass || shooter.inAllianceZone()) {
                  shooter.runDynamic(target, isPass);
                }
              }
            },
            shooter)
        .withName("ShootDynamic");
  }

  public static Command stop() {
    return Commands.sequence(
            Commands.runOnce(() -> RobotSystem.ShooterFunctions.shooterHold = true),
            Commands.runOnce(() -> shooter.stop(), shooter))
        .withName("ShooterStop");
  }
}
