package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotSystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.shotConfig;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LoggedTunableNumber;

public class ShooterCommands {
  private static final LoggedTunableNumber feederVolts =
      new LoggedTunableNumber("Shooter/feederVolts", 12);

  private static final LoggedTunableNumber spindexerVolts =
      new LoggedTunableNumber("Shooter/spindexerVolts", 10);
  // private static final double feederVolts = 12;
  // private static final double spindexerVolts = 12;
  private static final double flywheelBumpStep = 0.5;
  private static final double turretBumpStep = 1.0;

  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final Drive drive = robotSystem.getDrive();
  private static final Shooter shooter = robotSystem.getShooter();
  private static final RollerSystem spindexer = robotSystem.getSpindexer();
  private static final RollerSystem feeder = robotSystem.getFeeder();
  // Tunable parameters for spindexer overcurrent handling
  private static final LoggedTunableNumber spindexerOvercurrentThresholdLTN =
      new LoggedTunableNumber("Shooter/SpindexerOvercurrentThreshold", 60.0);
  private static final LoggedTunableNumber spindexerOvercurrentTimeLTN =
      new LoggedTunableNumber("Shooter/SpindexerOvercurrentTime", 0.4);
  private static final LoggedTunableNumber spindexerReverseVoltLTN =
      new LoggedTunableNumber("Shooter/SpindexerReverseVolt", -9.0);
  private static final LoggedTunableNumber spindexerReverseDurationLTN =
      new LoggedTunableNumber("Shooter/SpindexerReverseDuration", 0.25);

  @SuppressWarnings("unused")
  private static final LoggedTunableNumber spindexerReverseTestCurrent =
      new LoggedTunableNumber("Shooter/SpindexerReverseTestCurrent", 30);

  @SuppressWarnings("unused")
  private static final LoggedTunableNumber feederReverseTestCurrent =
      new LoggedTunableNumber("Shooter/FeederReverseTestCurrent", 30);

  private static boolean spindexerReverseActive = false;
  private static double spindexerReverseStart = 0.0;
  // Debouncer to detect sustained overcurrent (initialized with tunable time)
  private static Debouncer spindexerOvercurrentDebouncer =
      new Debouncer(spindexerOvercurrentTimeLTN.getAsDouble(), DebounceType.kRising);

  public static Command shootDefault() {
    return Commands.run(
            () -> {
              if (((drive.inAllianceZone() && HubShiftUtil.shouldShoot())
                      || (!drive.inAllianceZone() && robotSystem.autoshootPass.get()))
                  && !shooter.staticSetpoint) {
                robotSystem.isShooting = true;

                if (!shooter.holdSetpoint) {
                  if (shooter.atSetpoint() && shooter.inShootableLocation()) {
                    updateFeederSpindexer(true);
                  } else {
                    updateFeederSpindexer(false);
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
              // Reset spindexer overcurrent state on command end (use current tunable debounce)
              spindexerOvercurrentDebouncer =
                  new Debouncer(spindexerOvercurrentTimeLTN.getAsDouble(), DebounceType.kRising);
              spindexerReverseActive = false;
            })
        .withName("Shooter_Default");
  }

  public static Command shoot(boolean manualMode) {
    return Commands.run(
            () -> {
              if (!shooter.holdSetpoint) {
                if (drive.inAllianceZone() || !shooter.isTargetHub()) {
                  if (manualMode || shooter.forceManualShoot) {
                    updateFeederSpindexer(true);
                  } else {
                    if (shooter.atSetpoint()
                        && HubShiftUtil.shouldShoot()
                        && shooter.inShootableLocation()) {
                      updateFeederSpindexer(true);
                    } else {
                      updateFeederSpindexer(false);
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
              // Reset spindexer overcurrent state on command end (use current tunable debounce)
              spindexerOvercurrentDebouncer =
                  new Debouncer(spindexerOvercurrentTimeLTN.getAsDouble(), DebounceType.kRising);
              spindexerReverseActive = false;
            })
        .withName("Shooter_Shoot");
  }

  // Helper to centralize feeder/spindexer control including overcurrent reversal
  private static void updateFeederSpindexer(boolean runRequested) {
    if (runRequested) {
      double now = Timer.getFPGATimestamp();
      double spindexerCurrent = spindexer.getTorqueCurrent();
      double feederCurrent = feeder.getTorqueCurrent();
      // double spindexerCurrent =
      //     spindexerReverseTestCurrent.getAsDouble(); // TODO: remove after testing
      // double feederCurrent = feederReverseTestCurrent.getAsDouble(); // TODO: remove after
      // testing

      // Localize tunable values once per call
      double reverseDuration = spindexerReverseDurationLTN.getAsDouble();
      double reverseVolt = spindexerReverseVoltLTN.getAsDouble();
      double threshold = spindexerOvercurrentThresholdLTN.getAsDouble();
      double debounceTime = spindexerOvercurrentTimeLTN.getAsDouble();

      if (spindexerReverseActive) {
        if (now - spindexerReverseStart < reverseDuration) {
          spindexer.runVolts(reverseVolt);
          feeder.runVolts(reverseVolt);
          return;
        } else {
          spindexerReverseActive = false;
          spindexerOvercurrentDebouncer = new Debouncer(debounceTime, DebounceType.kRising);
          spindexer.runVolts(spindexerVolts.getAsDouble());
          feeder.runVolts(feederVolts.getAsDouble());
          return;
        }
      }

      // Not reversing: check for sustained overcurrent via debouncer
      // If debounce time changed, reinitialize debouncer so it uses the new time
      if (spindexerOvercurrentTimeLTN.hasChanged(ShooterCommands.class.hashCode())) {
        spindexerOvercurrentDebouncer = new Debouncer(debounceTime, DebounceType.kRising);
      }
      boolean debounced =
          spindexerOvercurrentDebouncer.calculate(
              spindexerCurrent > threshold || feederCurrent > threshold);
      if (debounced) {
        spindexerReverseActive = true;
        spindexerReverseStart = now;
        spindexer.runVolts(reverseVolt);
        feeder.runVolts(reverseVolt);
      } else {
        spindexer.runVolts(spindexerVolts.getAsDouble());
        feeder.runVolts(feederVolts.getAsDouble());
      }
    } else {
      feeder.stop();
      spindexer.stop();
      spindexerOvercurrentDebouncer =
          new Debouncer(spindexerOvercurrentTimeLTN.getAsDouble(), DebounceType.kRising);
      spindexerReverseActive = false;
    }
  }

  public static Command dontShoot() {
    return Commands.idle(feeder, spindexer).withName("Shooter_DontShoot");
  }

  public static Command runStatic(
      double flywheelVelocity, double hoodElevation, double turretPosition) {
    return Commands.runOnce(
            () -> shooter.runStatic(flywheelVelocity, hoodElevation, turretPosition), shooter)
        .withName("Shooter_Static");
  }

  public static Command runStatic(shotConfig config) {
    return runStatic(config.flywheelVelocity(), config.hoodAngle(), config.turretAngle());
  }

  public static Command stop() {
    return Commands.sequence(Commands.runOnce(() -> shooter.stop(), shooter))
        .withName("Shooter_Stop");
  }

  public static Command bumpFlywheel(boolean increaseSpeed) {
    return Commands.runOnce(
            () ->
                shooter.getFlywheel().bumpVelocity +=
                    (increaseSpeed) ? flywheelBumpStep : -flywheelBumpStep)
        .ignoringDisable(true)
        .withName("Shooter_BumpFlywheel");
  }

  public static Command bumpTurret(boolean increaseAngle) {
    return Commands.runOnce(
            () ->
                shooter.getTurret().bumpAngle += (increaseAngle) ? turretBumpStep : -turretBumpStep)
        .ignoringDisable(true)
        .withName("Shooter_BumpTurret");
  }

  public static Command setHoodForceDown(boolean value) {
    return Commands.runOnce(() -> shooter.getHood().setForceDown(value))
        .ignoringDisable(true)
        .withName("Shooter_SetHoodForceDown");
  }

  public static Command setHoldSetpoint(boolean value) {
    return Commands.runOnce(() -> shooter.holdSetpoint = value)
        .ignoringDisable(true)
        .withName("Shooter_SetHoldSetpoint");
  }

  /** Toggle the state of forceManualShoot. Optionally specify the value to set. */
  public static Command toggleManualShoot(boolean... value) {
    return Commands.runOnce(() -> shooter.forceManualShoot = !shooter.forceManualShoot)
        .onlyIf(() -> value.length == 0 || shooter.forceManualShoot != value[0])
        .ignoringDisable(true)
        .withName("Shooter_ToggleManualShoot");
  }

  public static Command clearStaticSetpoint() {
    return Commands.runOnce(() -> shooter.staticSetpoint = false)
        .ignoringDisable(true)
        .withName("Shooter_ClearStaticSetpoint");
  }
}
