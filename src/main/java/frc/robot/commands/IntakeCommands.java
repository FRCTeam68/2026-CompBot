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
            // TODO: Instead of running intakeSpin at 0 volts use the stop function. Ideally this
            // isn't even needed since any other command that uses intakeSpin should stop it before
            // finishing or being interupted. We do want to leave this here just in case though.
            Commands.runOnce(() -> intakeSpin.runVolts(0), intakeSpin),
            // TODO: We can remove the idle command since the intakeSpin does not need to remain on.
            // In fact we are turning it off in this command.
            Commands.idle())
        // TODO: We do not need to stop intakeSpin before this command ends. It isn't being run in
        // this command
        .finallyDo(() -> intakeSpin.stop())
        .withName("retractIntake");
  }

  public static Command intake() {
    return Commands.sequence(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.runOnce(() -> intakeSpin.runVolts(7), intakeSpin),
            Commands.idle())
        .finallyDo(() -> intakeSpin.stop())
        .withName("intake");
  }

  public static Command outtake() {
    return Commands.sequence(
            Commands.runOnce(
                () -> intakePivot.runPosition(IntakePivot.getExtended(), 0), intakePivot),
            Commands.runOnce(() -> intakeSpin.runVolts(-7), intakeSpin),
            Commands.idle())
        .finallyDo(() -> intakeSpin.stop())
        .withName("outtake");
  }
}
