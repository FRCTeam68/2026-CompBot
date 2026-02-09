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

  // TODO: add subsystem requirments to intake and outtake
  public static Command retract() {
    // TODO: This command is moving to the wrong position
    // TODO: We should give this a slightly more specific name. The climber and hood could also have
    // retract commands.
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
            // TODO: flip intake spin direction for outtake
            Commands.runOnce(() -> intakeSpin.runVolts(7)),
            Commands.idle())
        .finallyDo(() -> intakeSpin.stop())
        .withName("outtake");
  }
}
