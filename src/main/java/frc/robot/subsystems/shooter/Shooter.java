package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

// TODO: the shooter class needs to extend SubsystemBase to run the periodic method.
public class Shooter {
  @Getter private final Flywheel flywheel;
  @Getter private final Hood hood;
  @Getter private final Turret turret;

  public Shooter(Flywheel flywheel, Hood hood, Turret turret) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
  }

  public void periodic() {
    // TODO: call ShotVisualizer.Visualize if in simulation. We do not want the Rio to have to do
    // the processing on the real robot.
  }

  public void runStatic(double flywheelVelocity, double hoodElevation, double turretPosition) {
    // TODO: Finish this method
  }

  /** Stop motor */
  public void stop() {
    flywheel.stop();
    hood.stop();
    turret.stop();
  }

  // TODO: This is still logging to the MotorTemplate folder
  @AutoLogOutput(key = "MotorTemplate/atSetpoint")
  public boolean atSetpoint() {
    return flywheel.atSetpoint() && hood.atSetpoint() && turret.atSetpoint();
  }
}
