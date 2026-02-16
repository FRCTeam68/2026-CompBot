package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends SubsystemBase {
  @Getter private final Flywheel flywheel;
  @Getter private final Hood hood;
  @Getter private final Turret turret;

  public Shooter(Flywheel flywheel, Hood hood, Turret turret) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    SmartDashboard.putNumber("Shooter/FlywheelVelocity", 0);
    SmartDashboard.putNumber("Shooter/HoodPosition", 0);
    SmartDashboard.putNumber("Shooter/TurretPosition", 0);
    SmartDashboard.putData(
        "Shooter/RunStatic",
        Commands.runOnce(
                () ->
                    runStatic(
                        SmartDashboard.getNumber("Shooter/FlywheelVelocity", 0),
                        SmartDashboard.getNumber("Shooter/HoodPosition", 0),
                        SmartDashboard.getNumber("Shooter/TurretPosition", 0)))
            .withName("Shooter/RunStatic"));
  }

  public void periodic() {
    if (Constants.getMode() != Mode.REAL) {
      ShotVisualizer.visualize();
    }
  }

  public void runStatic(double flywheelVelocity, double hoodElevation, double turretPosition) {
    flywheel.runVelocity(flywheelVelocity, 0);
    hood.runElvation(hoodElevation, 0);
    turret.runPosition(turretPosition, 0);
  }

  /** Stop motor */
  public void stop() {
    flywheel.stop();
    hood.stop();
    turret.stop();
  }

  @AutoLogOutput(key = "Shooter/atSetpoint")
  public boolean atSetpoint() {
    return flywheel.atSetpoint() && hood.atSetpoint() && turret.atSetpoint();
  }
}
