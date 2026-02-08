package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotSystem;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.rollers.RollerSystem;

public class IntakeCommands { // Complete system
  private static final RobotSystem system = RobotSystem.getInstance();

  // Subsystems
  private static final IntakePivot intakePivot = system.getIntakePivot();
  private static final RollerSystem intakeSpin = system.getIntakeSpin();

  public static Command retract() {
    return Commands.runOnce(
            () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot)
        .withName("retract");
  }

  public static Command intake() {
    return Commands.sequence(
            Commands.runOnce(() -> intakePivot.runPosition(IntakePivot.getExtended(), 0)),
            Commands.runOnce(() -> intakeSpin.runVolts(7)),
            Commands.idle())
        .finallyDo(() -> intakeSpin.stop())
        .withName("intake");
  }

  public static Command outtake() {
    return Commands.sequence(
            Commands.runOnce(() -> intakePivot.runPosition(IntakePivot.getExtended(), 0)),
            Commands.runOnce(() -> intakeSpin.runVolts(7)),
            Commands.idle())
        .finallyDo(() -> intakeSpin.stop())
        .withName("outtake");
  }
}
