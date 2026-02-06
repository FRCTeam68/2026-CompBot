package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {
  public static Command shooter(Shooter shooter) {
    return Commands.sequence(Commands.runOnce(() -> shooter.runVelocity(1, 0)));
  }
}
