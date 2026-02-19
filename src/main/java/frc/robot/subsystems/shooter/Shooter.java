package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

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

    // TODO: You do not need to fix this, I do. This is causing a null requirment issue at boot.
    // SmartDashboard.putData(
    //     "Shooter/RunStatic",
    //     ShooterCommands.runStatic(
    //         SmartDashboard.getNumber("Shooter/FlywheelVelocity", 0.0),
    //         SmartDashboard.getNumber("Shooter/HoodPosition", 0.0),
    //         (SmartDashboard.getNumber("Shooter/TurretPosition", 0.0))));
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

  public void runDynamic(Translation2d target, boolean isPass) {
    double centerDistance =
        FieldConstants.Hub.innerCenterPoint
            .toTranslation2d()
            .minus(getShooterPose().getTranslation())
            .getNorm();
    double flightTime = ShooterConstants.hubShotFlightTime.get(centerDistance);
    target =
        target.plus(
            new Translation2d(
                driveVelocitySupplier.get().vxMetersPerSecond * flightTime * -1,
                driveVelocitySupplier.get().vyMetersPerSecond * flightTime * -1));
    runStatic(
        ShooterConstants.hubShotFlywheelVelocity.get(centerDistance),
        ShooterConstants.hubShotHoodElevation.get(centerDistance),
        // TODO: ** We are controlling the turrent with degrees not radians. Both spots need to be
        // changed.
        target.minus(getShooterPose().getTranslation()).getAngle().getRadians()
            - drivePoseSupplier.get().getRotation().getRadians());
  }

  public boolean inAllianceZone() {
    return AllianceFlipUtil.applyX(drivePoseSupplier.get().getX())
        < FieldConstants.LinesVertical.allianceZone + Units.inchesToMeters(23.5);
  }

  public Pose2d getShooterPose() {
    return new Pose2d(ShooterConstants.shooterPosition.toTranslation2d(), Rotation2d.kZero)
        .plus(
            new Transform2d(
                drivePoseSupplier.get().getTranslation(), drivePoseSupplier.get().getRotation()))
        .rotateAround(
            drivePoseSupplier.get().getTranslation(), drivePoseSupplier.get().getRotation());
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
