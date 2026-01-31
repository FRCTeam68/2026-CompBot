package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.RollerSystem;

public class IntakeCommands {
  public static Command intake(Intake intakePivot, RollerSystem intakeSpin) {
    return Commands.sequence(
            Commands.runOnce(() -> intakePivot.runPosition(Intake.getExtended(), 0)),
            Commands.runOnce(() -> intakeSpin.runVolts(7)),
            Commands.idle())
        .finallyDo(() -> intakeSpin.stop());
  }

  public static Command outtake(Intake intakePivot, RollerSystem intakeSpin) {
    return Commands.sequence(
            Commands.runOnce(() -> intakePivot.runPosition(Intake.getExtended(), 0)),
            Commands.runOnce(() -> intakeSpin.runVolts(7)),
            Commands.idle())
        .finallyDo(() -> intakeSpin.stop());
  }
}
