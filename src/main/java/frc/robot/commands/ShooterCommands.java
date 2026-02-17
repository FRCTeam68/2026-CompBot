package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotSystem;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {
  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final Shooter shooter = robotSystem.getShooter();
  private static final RollerSystem spindexer = robotSystem.getSpindexer();
  private static final RollerSystem feeder = robotSystem.getFeeder();

  // TODO: Add variables
  // static - shooter running at static speed
  // hold - keep shooter from changing
  // manual shoot toggle - manually shoot even when trying to auto shoot
  // no pass - keep shooter targeting hub shot to remove ramp up/down of flywheel

  // TODO: create shoot command to loop continually when scheduled
  // it should take in a boolean parameter for manual mode
  // if (not manual mode and not manual shoot toggle) {
  //   if (not hold) {
  //     if (shooter at setpoint) {
  //       run feeder
  //       run spindexer
  //     } else {
  //       stop feeder
  //       stop spindexer
  //     }
  //   } else {
  //     hold = false
  //   }
  // } else {
  //           run feeder
  //       run spindexer
  // }

  public static Command runStatic(
      double flywheelVelocity, double hoodElevation, double turretPosition) {
    // TODO: change static to true in command
    return Commands.sequence(
            Commands.runOnce(
                () -> shooter.runStatic(flywheelVelocity, hoodElevation, turretPosition), shooter))
        .withName("runStatic");
  }

  // TODO: change this method to run dynamic
  // It will only be used for commanding the shooter to move to the correct target
  public static Command autoShoot() {
    // create temp target variable // to store the target
    // create temp is pass variable // to store if we are trying to pass or not

    // if (not static and not hold) {
    //   // determine target
    //   if (in alliance zone or no pass) {
    //     target = hub
    //   } else {
    //     // still need to decide how to determine pass target
    //     // for not we can only target one point
    //     target = pass
    //   }

    //   if (is pass or in alliance zone) {
    //     call shooter run dynamic method and pass it the target
    //     // We will need to change the run dynamic method to accept a target
    //   }
    // }

    return Commands.run(
        () -> {
          shooter.runDynamic();
          if (shooter.atSetpoint()) {
            spindexer.runVolts(12);
            feeder.runVolts(12);
          }
        },
        null);
  }

  // TODO: create a command to stop the shooter
}
