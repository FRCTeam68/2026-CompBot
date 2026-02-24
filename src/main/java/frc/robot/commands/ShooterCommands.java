package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotSystem;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.shotConfig;
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

  // TODO: I split shooting into 2 commands. We need to fix the commands though
  public static Command shootAutomatic() {
    return Commands.run(
            () -> {
              if (!forceManualShoot) {
                if (!shooter.holdSetpoint) {
                  if (shooter.atSetpoint()) {
                    feeder.runVolts(feederRunVolts);
                    spindexer.runVolts(spindexerRunVolts);
                  } else {
                    feeder.stop();
                    spindexer.stop();
                  }
                } else {
                  shooter.holdSetpoint = false;
                }
              } else {
                feeder.runVolts(feederRunVolts);
                spindexer.runVolts(spindexerRunVolts);
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

  public static Command shootManual() {
    return Commands.none();
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

  // TODO: create a command to toggle manual shoot
}
