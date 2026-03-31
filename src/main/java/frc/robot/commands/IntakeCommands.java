package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotSystem;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakeSpin.IntakeSpin;
import frc.robot.subsystems.sensors.HopperSensor;
import frc.robot.util.LoggedTunableNumber;

public class IntakeCommands {
  private static final LoggedTunableNumber intakeSpinVoltsSlow =
      new LoggedTunableNumber("IntakeSpin/Slow", 6);
  private static final LoggedTunableNumber intakeSpinVoltsFast =
      new LoggedTunableNumber("IntakeSpin/Fast", 10);
  private static final double intakeSpinVoltsDefault = 8;
  private static final LoggedTunableNumber intakeSpinVoltsOuttake =
      new LoggedTunableNumber("IntakeSpin/Outtake", -10);

  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final IntakePivot intakePivot = robotSystem.getIntakePivot();
  private static final IntakeSpin intakeSpin = robotSystem.getIntakeSpin();
  private static final HopperSensor hopperSensor = robotSystem.getHopperSensor();

  public static Command deploy(int slot) {
    return Commands.parallel(
            stopSpin(),
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), slot), intakePivot))
        .withName("Intake_Deploy");
  }

  public static Command retract() {
    return Commands.parallel(
            stopSpin(),
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getPackaged(), 1), intakePivot))
        .withName("Intake_Retract");
  }

  public static Command agitate(double... timeout) {
    final double waitTime = (timeout.length == 0) ? 0.0 : timeout[0];
    return Commands.sequence(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getAgitate(), 1), intakePivot),
            Commands.runOnce(() -> intakeSpin.runVolts(4), intakeSpin),
            Commands.either(
                Commands.idle(intakePivot, intakeSpin),
                Commands.waitSeconds(waitTime),
                () -> timeout.length == 0))
        .finallyDo(
            () -> {
              intakePivot.runPosition(IntakePivot.getExtended(), 0);
              intakeSpin.stop();
            })
        .withName("Intake_Agitate");
  }

  public static Command intakeDefault() {
    return Commands.sequence(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.run(
                () -> {
                  if (robotSystem.autoIntake.get()) {
                    if (hopperSensor.isConnected()) {
                      if (hopperSensor.isNotEmpty()) {
                        intakeSpin.runVolts(intakeSpinVoltsFast.get());
                      } else {
                        intakeSpin.runVolts(intakeSpinVoltsSlow.get());
                      }
                    } else {
                      intakeSpin.runVolts(intakeSpinVoltsDefault);
                    }
                  }
                },
                intakeSpin))
        .finallyDo(() -> intakeSpin.stop())
        .withName("Intake_Default");
  }

  public static Command intakeStatic(boolean slowMode) {
    return Commands.sequence(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.runOnce(
                () ->
                    intakeSpin.runVolts(
                        slowMode ? intakeSpinVoltsSlow.get() : intakeSpinVoltsFast.get()),
                intakeSpin))
        .withName("Intake_Static");
  }

  public static Command intakeAutomatic() {
    return Commands.parallel(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.run(
                () -> {
                  if (hopperSensor.isConnected()) {
                    if (hopperSensor.isNotEmpty()) {
                      intakeSpin.runVolts(intakeSpinVoltsFast.get());
                    } else {
                      intakeSpin.runVolts(intakeSpinVoltsSlow.get());
                    }
                  } else {
                    intakeSpin.runVolts(intakeSpinVoltsDefault);
                  }
                },
                intakeSpin))
        .finallyDo(() -> intakeSpin.stop())
        .withName("Intake_Automatic");
  }

  public static Command outtake() {
    return Commands.parallel(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.runOnce(() -> intakeSpin.runVolts(intakeSpinVoltsOuttake.get()), intakeSpin),
            Commands.idle())
        .finallyDo(() -> intakeSpin.stop())
        .withName("Intake_Outtake");
  }

  public static Command dontIntake() {
    return Commands.idle(intakeSpin).withName("Intake_DontIntake");
  }

  public static Command stopSpin() {
    return Commands.runOnce(() -> intakeSpin.stop(), intakeSpin).withName("Intake_StopSpin");
  }
}
