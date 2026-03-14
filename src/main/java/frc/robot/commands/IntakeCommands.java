package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotSystem;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.rollers.RollerSystem;

public class IntakeCommands {
  private static final double intakeSpinVolts = 8;

  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final IntakePivot intakePivot = robotSystem.getIntakePivot();
  private static final RollerSystem intakeSpin = robotSystem.getIntakeSpin();

  public static Command retract() {
    return Commands.sequence(
            stopSpin(),
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getPackaged(), 1), intakePivot))
        .withName("IntakeRetract");
  }

  public static Command agitate(double... timeout) {
    double waitTime = (timeout.length == 0) ? 0.0 : timeout[0];
    return Commands.sequence(
            stopSpin(),
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getAgitate(), 1), intakePivot),
            Commands.either(
                Commands.idle(intakePivot, intakeSpin),
                Commands.waitSeconds(waitTime),
                () -> timeout.length == 0))
        .finallyDo(() -> intakePivot.runPosition(IntakePivot.getExtended(), 0))
        .withName("IntakeAgitate");
  }

  public static Command intakeRemainOn() {
    return Commands.sequence(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.waitUntil(() -> intakePivot.atSetpoint()),
            Commands.runOnce(() -> intakeSpin.runVolts(intakeSpinVolts), intakeSpin))
        .withName("IntakeOn");
  }

  public static Command intakeWhile() {
    return Commands.sequence(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.waitUntil(() -> intakePivot.atSetpoint()),
            Commands.runOnce(() -> intakeSpin.runVolts(intakeSpinVolts), intakeSpin),
            Commands.idle())
        .finallyDo(() -> intakeSpin.stop())
        .withName("IntakeWhile");
  }

  public static Command outtake() {
    return Commands.sequence(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.waitUntil(() -> intakePivot.atSetpoint()),
            Commands.runOnce(() -> intakeSpin.runVolts(-intakeSpinVolts), intakeSpin),
            Commands.idle())
        .finallyDo(() -> intakeSpin.stop())
        .withName("Outtake");
  }

  public static Command stopSpin() {
    return Commands.runOnce(() -> intakeSpin.stop(), intakeSpin).withName("IntakeSpinStop");
  }
}
