package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import frc.robot.subsystems.shooter.ShooterConstants.shotConfig;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  // Subsystems
  @Getter private final Flywheel flywheel;
  @Getter private final Hood hood;
  @Getter private final Turret turret;

  private Translation2d target = Translation2d.kZero;
  @Getter private boolean isTargetHub = true;

  @AutoLogOutput(key = "Shooter/HoldSetpoint")
  public boolean holdSetpoint = false;

  @AutoLogOutput(key = "Shooter/NoPass")
  public boolean noPass = false;

  private final Supplier<Pose2d> drivePoseSupplier;
  private final Supplier<ChassisSpeeds> driveVelocitySupplier;
  private final Supplier<Boolean> inAllianceZoneSupplier;

  public Shooter(
      Flywheel flywheel,
      Hood hood,
      Turret turret,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> driveVelocitySupplier,
      Supplier<Boolean> inAllianceZoneSupplier) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    this.drivePoseSupplier = poseSupplier;
    this.driveVelocitySupplier = driveVelocitySupplier;
    this.inAllianceZoneSupplier = inAllianceZoneSupplier;

    // Configure dashboard
    SmartDashboard.putNumber("Shooter/FlywheelVelocity", 0.0);
    SmartDashboard.putNumber("Shooter/HoodPosition", 0.0);
    SmartDashboard.putNumber("Shooter/TurretPosition", 0.0);
    SmartDashboard.putData(
        "Shooter/RunStatic",
        Commands.runOnce(
            () -> {
              runStatic(
                  SmartDashboard.getNumber("Shooter/FlywheelVelocity", 0.0),
                  SmartDashboard.getNumber("Shooter/HoodPosition", 0.0),
                  SmartDashboard.getNumber("Shooter/TurretPosition", 0.0));
            },
            this));

    SmartDashboard.putData("Shooter/Tuning/1", Commands.runOnce(() -> runStatic(0, 0, 1)));
    SmartDashboard.putData("Shooter/Tuning/360", Commands.runOnce(() -> runStatic(0, 0, 360)));
  }

  public void periodic() {
    // Log shot visualizer if sim
    if (Constants.getMode() != Mode.REAL) {
      ShotVisualizer.visualize();
    }

    // Calculate target
    if (inAllianceZoneSupplier.get() || noPass) {
      isTargetHub = true;
      target = AllianceFlipUtil.apply(ShooterConstants.Target.hub);
      Logger.recordOutput("Shooter/Target", "Hub");
    } else {
      isTargetHub = false;
      if (drivePoseSupplier.get().getY() < FieldConstants.LinesHorizontal.center) {
        target =
            AllianceFlipUtil.apply(
                (AllianceFlipUtil.shouldFlip())
                    ? ShooterConstants.Target.passLeft
                    : ShooterConstants.Target.passRight);
        Logger.recordOutput("Shooter/Target", "Pass Right");
      } else {
        target =
            AllianceFlipUtil.apply(
                (AllianceFlipUtil.shouldFlip())
                    ? ShooterConstants.Target.passRight
                    : ShooterConstants.Target.passLeft);
        Logger.recordOutput("Shooter/Target", "Pass Left");
      }
    }

    // Run shooter to target
    if (!holdSetpoint && (!isTargetHub || inAllianceZoneSupplier.get())) {
      runDynamic();
    }
  }

  /**
   * Run the shooter at a static speed and position.
   *
   * @param flywheelVelocity The velocity of the flywheel in rotations per second.
   * @param hoodElevation The elevation of the hood in degrees.
   * @param turretAngle The counterclockwise angle of the turret in degrees.
   */
  public void runStatic(double flywheelVelocity, double hoodElevation, double turretAngle) {
    holdSetpoint = true;
    flywheel.runVelocity(flywheelVelocity, 0);
    hood.runElvation(hoodElevation, 0);
    turret.runPosition(turretAngle, 0);
  }

  /**
   * Run the shooter at a static speed and position.
   *
   * @param config The goal shooter configuration.
   */
  public void runStatic(shotConfig config) {
    runStatic(config.flywheelVelocity(), config.hoodAngle(), config.turretAngle());
  }

  /** Run the shooter dynamically. */
  public void runDynamic() {
    double targetDistance = target.minus(getShooterFieldTranslation()).getNorm();
    double flightTime = ShooterConstants.DynamicShot.hubShotFlightTime.get(targetDistance);
    Translation2d adjustedTarget = target;
    // target.plus(
    //     new Translation2d(
    //         driveVelocitySupplier.get().vxMetersPerSecond * flightTime * -1,
    //         driveVelocitySupplier.get().vyMetersPerSecond * flightTime * -1));
    double adjustedTargetDistance = target.minus(getShooterFieldTranslation()).getNorm();

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
        adjustedTarget.minus(getShooterFieldTranslation()).getAngle().getDegrees()
            - drivePoseSupplier.get().getRotation().getDegrees(),
        0);
  }

  /** Stop all shooter subsytems. */
  public void stop() {
    holdSetpoint = true;
    flywheel.stop();
    hood.stop();
    turret.stop();
  }

  /** Returns the the field relative position of the shooter. */
  @AutoLogOutput
  public Translation2d getShooterFieldTranslation() {
    return drivePoseSupplier
        .get()
        .getTranslation()
        .plus(ShooterConstants.shooterPosition.toTranslation2d())
        .rotateAround(
            drivePoseSupplier.get().getTranslation(), drivePoseSupplier.get().getRotation());
  }

  /**
   * Checks if the shooter is near any of the trenches. If so, the hood should be forced down to
   * avoid collisions.
   *
   * @return If the shooter is near the trench.
   */
  @AutoLogOutput(key = "Shooter/InTrenchBox")
  public boolean inTrenchBox() {
    Translation2d shooterTranslation = getShooterFieldTranslation();

    // The maximum time for the hood to lower to underTrenchMinimum
    double hoodLowerTime = 0.5;

    // Default box size. xMin should be set big enough to allow ample time for the hood to go down
    // from 0 velocity.
    double xSize = Units.inchesToMeters(47);
    double ySize = FieldConstants.LinesHorizontal.rightTrenchOpenStart;

    // Adjust x limits based on velocity
    double xOffestPos =
        (xSize / 2) + (-Math.min(0, driveVelocitySupplier.get().vxMetersPerSecond) * hoodLowerTime);
    double xOffsetNeg =
        (-xSize / 2) - (Math.max(0, driveVelocitySupplier.get().vxMetersPerSecond) * hoodLowerTime);

    // Check y position
    return (shooterTranslation.getY() < ySize
            || shooterTranslation.getY() > FieldConstants.fieldWidth - ySize)
        // Check blue alliance x position
        && ((shooterTranslation.getX() < FieldConstants.LinesVertical.hubCenter + xOffestPos
                && shooterTranslation.getX() > FieldConstants.LinesVertical.hubCenter + xOffsetNeg)
            // Check red alliance x position
            || (shooterTranslation.getX() < FieldConstants.LinesVertical.oppHubCenter + xOffestPos
                && shooterTranslation.getX()
                    > FieldConstants.LinesVertical.oppHubCenter + xOffsetNeg));
  }

  @AutoLogOutput(key = "Shooter/DistanceToTarget")
  public double getDistanceToTarget() {
    return drivePoseSupplier.get().getTranslation().minus(target).getNorm();
  }

  /** Returns true if all shooter subsystems are at their individual setpoints. */
  @AutoLogOutput(key = "Shooter/atSetpoint")
  public boolean atSetpoint() {
    return flywheel.atSetpoint() && hood.atSetpoint() && turret.atSetpoint();
  }
}
