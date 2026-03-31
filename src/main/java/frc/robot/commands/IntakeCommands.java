package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotSystem;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.rollers.RollerSystem;
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
  private static final RollerSystem intakeSpin1 = robotSystem.getIntakeSpin1();
  private static final RollerSystem intakeSpin2 = robotSystem.getIntakeSpin2();
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
            Commands.runOnce(() -> intakeSpin1.runVolts(4), intakeSpin1),
            Commands.runOnce(() -> intakeSpin2.runVolts(4), intakeSpin2),
            Commands.either(
                Commands.idle(intakePivot, intakeSpin1, intakeSpin2),
                Commands.waitSeconds(waitTime),
                () -> timeout.length == 0))
        .finallyDo(
            () -> {
              intakePivot.runPosition(IntakePivot.getExtended(), 0);
              intakeSpin1.stop();
              intakeSpin2.stop();
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
                        intakeSpin1.runVolts(intakeSpinVoltsFast.get());
                        intakeSpin2.runVolts(intakeSpinVoltsFast.get());
                      } else {
                        intakeSpin1.runVolts(intakeSpinVoltsSlow.get());
                        intakeSpin2.runVolts(intakeSpinVoltsSlow.get());
                      }
                    } else {
                      intakeSpin1.runVolts(intakeSpinVoltsDefault);
                      intakeSpin2.runVolts(intakeSpinVoltsDefault);
                    }
                  }
                },
                intakeSpin1,
                intakeSpin2))
        .finallyDo(
            () -> {
              intakeSpin1.stop();
              intakeSpin2.stop();
            })
        .withName("Intake_Default");
  }

  public static Command intakeStatic(boolean slowMode) {
    return Commands.sequence(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.runOnce(
                () ->
                    intakeSpin1.runVolts(
                        slowMode ? intakeSpinVoltsSlow.get() : intakeSpinVoltsFast.get()),
                intakeSpin1),
            Commands.runOnce(
                () ->
                    intakeSpin2.runVolts(
                        slowMode ? intakeSpinVoltsSlow.get() : intakeSpinVoltsFast.get()),
                intakeSpin2))
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
                      intakeSpin1.runVolts(intakeSpinVoltsFast.get());
                      intakeSpin2.runVolts(intakeSpinVoltsFast.get());
                    } else {
                      intakeSpin1.runVolts(intakeSpinVoltsSlow.get());
                      intakeSpin2.runVolts(intakeSpinVoltsSlow.get());
                    }
                  } else {
                    intakeSpin1.runVolts(intakeSpinVoltsDefault);
                    intakeSpin2.runVolts(intakeSpinVoltsDefault);
                  }
                },
                intakeSpin1,
                intakeSpin2))
        .finallyDo(
            () -> {
              intakeSpin1.stop();
              intakeSpin2.stop();
            })
        .withName("Intake_Automatic");
  }

  public static Command outtake() {
    return Commands.parallel(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.runOnce(() -> intakeSpin1.runVolts(intakeSpinVoltsOuttake.get()), intakeSpin1),
            Commands.runOnce(() -> intakeSpin2.runVolts(intakeSpinVoltsOuttake.get()), intakeSpin2),
            Commands.idle())
        .finallyDo(
            () -> {
              intakeSpin1.stop();
              intakeSpin2.stop();
            })
        .withName("Intake_Outtake");
  }

  public static Command dontIntake() {
    return Commands.idle(intakeSpin1, intakeSpin2).withName("Intake_DontIntake");
  }

  public static Command stopSpin() {
    return Commands.runOnce(
            () -> {
              intakeSpin1.stop();
              intakeSpin2.stop();
            },
            intakeSpin1,
            intakeSpin2)
        .withName("Intake_StopSpin");
  }
}
