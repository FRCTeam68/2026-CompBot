package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotSystem;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.rollers.RollerSystem;

public class IntakeCommands {
  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final IntakePivot intakePivot = robotSystem.getIntakePivot();
  private static final RollerSystem intakeSpin = robotSystem.getIntakeSpin();

  public static Command retract() {
    return Commands.sequence(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getPackaged(), 0), intakePivot),
            Commands.runOnce(() -> intakeSpin.stop(), intakeSpin))
        .withName("IntakeRetract");
  }

  public static Command intakeOn() {
    return Commands.sequence(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.runOnce(() -> intakeSpin.runVolts(7), intakeSpin))
        .withName("IntakeOn");
  }

  public static Command intakeWhile() {
    return Commands.sequence(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.runOnce(() -> intakeSpin.runVolts(7), intakeSpin),
            Commands.idle())
        .finallyDo(() -> intakeSpin.stop())
        .withName("IntakeWhile");
  }

  public static Command outtake() {
    return Commands.sequence(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.runOnce(() -> intakeSpin.runVolts(-7), intakeSpin),
            Commands.idle())
        .finallyDo(() -> intakeSpin.stop())
        .withName("Outtake");
  }

  public static Command stop() {
    return Commands.runOnce(() -> intakeSpin.stop(), intakeSpin).withName("IntakeSpinStop");
  }
}
