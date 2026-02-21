package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  @Getter private final Flywheel flywheel;
  @Getter private final Hood hood;
  @Getter private final Turret turret;
  private final Supplier<Pose2d> drivePoseSupplier;
  private final Supplier<ChassisSpeeds> driveVelocitySupplier;

  public Shooter(
      Flywheel flywheel,
      Hood hood,
      Turret turret,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> driveVelocitySupplier) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    this.drivePoseSupplier = poseSupplier;
    this.driveVelocitySupplier = driveVelocitySupplier;

    SmartDashboard.putNumber("Shooter/FlywheelVelocity", 0.0);
    SmartDashboard.putNumber("Shooter/HoodPosition", 0.0);
    SmartDashboard.putNumber("Shooter/TurretPosition", 0.0);
    SmartDashboard.putData(
        "Shooter/RunStatic",
        Commands.runOnce(
            () -> {
              ShooterCommands.shooterHold = true;
              runStatic(
                  SmartDashboard.getNumber("Shooter/FlywheelVelocity", 0.0),
                  SmartDashboard.getNumber("Shooter/HoodPosition", 0.0),
                  SmartDashboard.getNumber("Shooter/TurretPosition", 0.0));
            }));
  }

  public void periodic() {
    if (Constants.getMode() != Mode.REAL) {
      ShotVisualizer.visualize();
    }
    Logger.recordOutput(
        "Shooter/Distance",
        FieldConstants.Hub.innerCenterPoint
            .toTranslation2d()
            .minus(getFieldShooterPose().getTranslation())
            .getNorm());
  }

  /**
   * Run the shooter at a static speed and position.
   *
   * @param flywheelVelocity The velocity of the flywheel in rotations per second.
   * @param hoodElevation The elevation of the hood in degrees.
   * @param turretAngle The counterclockwise angle of the turret in degrees.
   */
  public void runStatic(double flywheelVelocity, double hoodElevation, double turretAngle) {
    flywheel.runVelocity(flywheelVelocity, 0);
    hood.runElvation(hoodElevation, 0);
    turret.runPosition(turretAngle, 0);
  }

  // TODO: create a runStatic method that takes a SotConfig. Use the following doc-comment.
  /**
   * Run the shooter at a static speed and position.
   *
   * @param shotConfig The goal shooter configuration.
   */

  /**
   * Run the shooter dynamically.
   *
   * @param target Position of the shot target.
   * @param isPass True to use pass shot config. Otherwise, hub shot config is assumed.
   */
  public void runDynamic(Translation2d target, boolean isPass) {
    double targetDistance = target.minus(getFieldShooterPose().getTranslation()).getNorm();
    double flightTime = ShooterConstants.DynamicShot.hubShotFlightTime.get(targetDistance);
    Translation2d adjustedTarget =
        target.plus(
            new Translation2d(
                driveVelocitySupplier.get().vxMetersPerSecond * flightTime * -1,
                driveVelocitySupplier.get().vyMetersPerSecond * flightTime * -1));
    double adjustedTargetDistance = target.minus(getFieldShooterPose().getTranslation()).getNorm();

    Logger.recordOutput(
        "Shooter/AdjustedTarget",
        new Pose3d(
            new Translation3d(adjustedTarget).plus(new Translation3d(0.0, 0.0, 1.8)),
            Rotation3d.kZero));

    flywheel.runVelocity(
        ShooterConstants.DynamicShot.hubShotFlywheelVelocity.get(adjustedTargetDistance), 0);
    hood.runElvation(
        ShooterConstants.DynamicShot.hubShotHoodElevation.get(adjustedTargetDistance), 0);
    turret.runPosition(
        adjustedTarget.minus(getFieldShooterPose().getTranslation()).getAngle().getDegrees()
            - drivePoseSupplier.get().getRotation().getDegrees(),
        0);
  }

  /**
   * Returns if the bumpers are in the alliance zone. This check is approximate and does not take
   * into account the chassis rotation.
   */
  public boolean inAllianceZone() {
    return AllianceFlipUtil.applyX(drivePoseSupplier.get().getX())
        < FieldConstants.LinesVertical.allianceZone + Units.inchesToMeters(23.5);
  }

  /** Returns the the field relative position of the shooter. */
  public Pose2d getFieldShooterPose() {
    return new Pose2d(ShooterConstants.shooterPosition.toTranslation2d(), Rotation2d.kZero)
        .plus(
            new Transform2d(
                drivePoseSupplier.get().getTranslation(),
                new Rotation2d(Units.degreesToRadians(turret.getPosition()))))
        .rotateAround(
            drivePoseSupplier.get().getTranslation(), drivePoseSupplier.get().getRotation());
  }

  /** Stop all shooter subsytems. */
  public void stop() {
    flywheel.stop();
    hood.stop();
    turret.stop();
  }

  /** Returns true if all shooter subsystems are at their individual setpoints. */
  @AutoLogOutput(key = "Shooter/atSetpoint")
  public boolean atSetpoint() {
    return flywheel.atSetpoint() && hood.atSetpoint() && turret.atSetpoint();
  }
}
