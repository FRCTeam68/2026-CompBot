package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotSystem;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {
  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final Shooter shooter = robotSystem.getShooter();
  private static final RollerSystem spindexer = robotSystem.getSpindexer();
  private static final RollerSystem feeder = robotSystem.getFeeder();

  private static boolean staticShooterSpeed = false;
  private static boolean shooterHold = false;
  private static boolean manualShootToggle = false;
  private static boolean noPass = true;

  public static Command shootLoop(boolean manual) {
    // TODO: when this command is interupted it should stop the feeder and spindexer
    return Commands.run(
            () -> {
              if (!manual && !manualShootToggle) {
                if (!shooterHold) {
                  if (shooter.atSetpoint()) {
                    feeder.runVolts(12);
                    spindexer.runVolts(12);
                  } else {
                    feeder.stop();
                    spindexer.stop();
                  }
                } else {
                  shooterHold = false;
                }
              } else {
                feeder.runVolts(12);
                spindexer.runVolts(12);
              }
            },
            feeder,
            spindexer)
        .withName("ShootLoop");
  }

  public static Command runStatic(
      double flywheelVelocity, double hoodElevation, double turretPosition) {
    return Commands.sequence(
            Commands.runOnce(() -> staticShooterSpeed = true),
            Commands.runOnce(
                () -> shooter.runStatic(flywheelVelocity, hoodElevation, turretPosition), shooter))
        .withName("RunStatic");
  }

  public static Command runDynamic() {
    // TODO: This command must have it's subsytem requirment added
    return Commands.run(
            () -> {
              Translation2d target;
              boolean isPass;
              if (!staticShooterSpeed && !shooterHold) {
                if (shooter.inAllianceZone() || noPass) {
                  target = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
                  isPass = false;
                } else {
                  target = new Translation2d();
                  isPass = true;
                }

                if (isPass || shooter.inAllianceZone()) {
                  shooter.runDynamic(target, isPass);
                }
              }
            })
        .withName("runDynamic");
  }

  // TODO: create a command to stop the shooter
}
