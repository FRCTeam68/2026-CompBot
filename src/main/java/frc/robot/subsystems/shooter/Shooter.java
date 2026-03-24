package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.util.LoggedTunableNumber;
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

  // Suppliers
  private final Supplier<Pose2d> drivePoseSupplier;
  private final Supplier<ChassisSpeeds> driveVelocitySupplier;
  private final Supplier<Boolean> inAllianceZoneSupplier;
  private final Supplier<Boolean> alwaysTargetPass;
  private final Supplier<Boolean> autoshootPass;

  private Translation2d target = Translation2d.kZero;
  @Getter private boolean isTargetHub = true;
  @Getter private double flightTime = 0.0;
  public boolean shouldTargetPass = false;

  @AutoLogOutput(key = "Shooter/HoldSetpoint")
  public boolean holdSetpoint = false;

  @AutoLogOutput(key = "Shooter/StaticSetpoint")
  public boolean staticSetpoint = false;

  @AutoLogOutput(key = "Shooter/ManualShoot")
  public boolean forceManualShoot = false;

  public static final LoggedTunableNumber towardMultiplier =
      new LoggedTunableNumber("Shooter/TowardMultiplier", 1.1);
  public static final LoggedTunableNumber awayMultiplier =
      new LoggedTunableNumber("Shooter/AwayMultiplier", 1.2);
  public static final LoggedTunableNumber angleMultiplier =
      new LoggedTunableNumber("Shooter/AngleMultiplier", 1.05);

  public Shooter(
      Flywheel flywheel,
      Hood hood,
      Turret turret,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> driveVelocitySupplier,
      Supplier<Boolean> inAllianceZoneSupplier,
      Supplier<Boolean> alwaysTargetPass,
      Supplier<Boolean> autoshootPass) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    this.drivePoseSupplier = poseSupplier;
    this.driveVelocitySupplier = driveVelocitySupplier;
    this.inAllianceZoneSupplier = inAllianceZoneSupplier;
    this.alwaysTargetPass = alwaysTargetPass;
    this.autoshootPass = autoshootPass;

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
  }

  public void periodic() {
    // Calculate target
    if (inAllianceZoneSupplier.get()) {
      isTargetHub = true;
      holdSetpoint = false;
      target = AllianceFlipUtil.apply(ShooterConstants.Target.hub);
      flightTime = ShooterConstants.DynamicShot.hubShotFlightTime.get(getDistanceToTarget());
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
      flightTime = ShooterConstants.DynamicShot.passShotFlightTime.get(getDistanceToTarget());
    }

    // Run shooter to target dynamically
    if (!staticSetpoint && !holdSetpoint) {
      if (inAllianceZoneSupplier.get()
          || autoshootPass.get()
          || (!DriverStation.isAutonomous() && (shouldTargetPass || alwaysTargetPass.get()))) {
        runDynamic();
      } else {
        // If not actively targeting lower hood
        hood.runElvation(Hood.getUnderTrenchMinimum());
        holdSetpoint = true;
      }
    }

    // Log shot visualizer if not running on the real robot
    if (Constants.getMode() != Mode.REAL) {
      ShotVisualizer.visualize();
    }

    // Log distance to target to the dashdoard with 3 decimal places
    // Only do this in tuning mode since String.format is resource intensive
    // The distance is still available in the log
    if (Constants.tuningMode) {
      SmartDashboard.putString(
          "Shooter/DistanceToTarget", String.format("%.3f", getDistanceToTarget()));
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
    staticSetpoint = true;
    flywheel.runBangBang(flywheelVelocity);
    hood.runElvation(hoodElevation);
    turret.runPosition(turretAngle);
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
    final Rotation2d rotationToTarget = target.minus(getShooterFieldTranslation()).getAngle();
    // vx - toward target
    // vy - CW tangent to target
    final ChassisSpeeds targetRelativeVelocity =
        ChassisSpeeds.fromFieldRelativeSpeeds(driveVelocitySupplier.get(), rotationToTarget);
    final double linearMultiplier =
        targetRelativeVelocity.vxMetersPerSecond > 0
            ? towardMultiplier.get()
            : awayMultiplier.get();
    final double targetDistanceAdjusted =
        getDistanceToTarget()
            - (targetRelativeVelocity.vxMetersPerSecond * flightTime * linearMultiplier);

    flywheel.runBangBang(
        ShooterConstants.DynamicShot.hubShotFlywheelVelocity.get(targetDistanceAdjusted));
    hood.runElvation(ShooterConstants.DynamicShot.hubShotHoodElevation.get(targetDistanceAdjusted));
    turret.runPosition(
        rotationToTarget
            .minus(
                new Translation2d(
                        getDistanceToTarget(),
                        targetRelativeVelocity.vyMetersPerSecond
                            * flightTime
                            * angleMultiplier.get())
                    .getAngle())
            .minus(drivePoseSupplier.get().getRotation())
            .getDegrees());
  }

  /** Stop all shooter subsytems. */
  public void stop() {
    holdSetpoint = true;
    staticSetpoint = false;
    flywheel.stop();
    hood.stop();
    turret.stop();
  }

  /** Returns the the field relative position of the shooter. */
  public Translation2d getShooterFieldTranslation() {
    return drivePoseSupplier
        .get()
        .getTranslation()
        .plus(ShooterConstants.shooterPosition.toTranslation2d())
        .rotateAround(
            drivePoseSupplier.get().getTranslation(), drivePoseSupplier.get().getRotation());
  }

  /** Returns the distance in meters to the automatically selected target. */
  @AutoLogOutput(key = "Shooter/DistanceToTarget", unit = "Meters")
  public double getDistanceToTarget() {
    return target.minus(getShooterFieldTranslation()).getNorm();
  }

  /** Returns true if all shooter subsystems are at their individual setpoints. */
  @AutoLogOutput(key = "Shooter/atSetpoint")
  public boolean atSetpoint() {
    return flywheel.atSetpoint() && hood.atSetpoint() && turret.atSetpoint();
  }

  /**
   * Returns if the position of the shooter is in a spot where we can shoot from.
   *
   * <p><b>Hub Shot:</b>
   *
   * <ul>
   *   <li>Not too close to the hub
   *   <li>Not in trench box
   *   <li>Not behind tower
   * </ul>
   *
   * <p><b>Pass Shot:</b>
   *
   * <ul>
   *   <li>Not in trench box
   *   <li>Not behind tower
   *   <li>Not behind hub
   * </ul>
   */
  public boolean inShootableLocation() {
    if (isTargetHub) {
      return getDistanceToTarget() > ShooterConstants.DynamicShot.minHubShotDistance
          && !inTrenchBox()
          && !inTowerBox();
    } else {
      return !inTrenchBox() && !inTowerBox() && !isBehindHub();
    }
  }

  /**
   * returns if the shooter is near any of the trenches. The size of the box is proportional to the
   * robots velocity. If so, the hood should be forced down to avoid collisions.
   */
  @AutoLogOutput(key = "Shooter/InTrenchBox")
  public boolean inTrenchBox() {
    final Translation2d shooterTranslation = getShooterFieldTranslation();

    // Adjust x limits based on velocity
    final double xOffestPos =
        (ShooterConstants.TrenchZone.halfXSize)
            + (-Math.min(0, driveVelocitySupplier.get().vxMetersPerSecond)
                * ShooterConstants.TrenchZone.hoodLowerTime);
    final double xOffsetNeg =
        (-ShooterConstants.TrenchZone.halfXSize)
            - (Math.max(0, driveVelocitySupplier.get().vxMetersPerSecond)
                * ShooterConstants.TrenchZone.hoodLowerTime);

    // Check y position
    return (shooterTranslation.getY() < ShooterConstants.TrenchZone.ySize
            || shooterTranslation.getY()
                > FieldConstants.fieldWidth - ShooterConstants.TrenchZone.ySize)
        // Check blue alliance x position
        && ((shooterTranslation.getX() < FieldConstants.LinesVertical.hubCenter + xOffestPos
                && shooterTranslation.getX() > FieldConstants.LinesVertical.hubCenter + xOffsetNeg)
            // Check red alliance x position
            || (shooterTranslation.getX() < FieldConstants.LinesVertical.oppHubCenter + xOffestPos
                && shooterTranslation.getX()
                    > FieldConstants.LinesVertical.oppHubCenter + xOffsetNeg));
  }

  /** Returns if the shooter is inside any tower. */
  @AutoLogOutput(key = "Shooter/InTowerBox")
  public boolean inTowerBox() {
    final Translation2d shooterTranslation = getShooterFieldTranslation();

    // Check blue alliance
    return (shooterTranslation.getX() < ShooterConstants.TowerZone.xSize
            && Math.abs(
                    shooterTranslation.getY()
                        - ShooterConstants.TowerZone.yMin
                        - ShooterConstants.TowerZone.halfYSize)
                < ShooterConstants.TowerZone.halfYSize)
        // Check red alliance
        || (shooterTranslation.getX()
                > FieldConstants.fieldLength - ShooterConstants.TowerZone.xSize
            && Math.abs(
                    FieldConstants.fieldWidth
                        - shooterTranslation.getY()
                        - ShooterConstants.TowerZone.yMin
                        - ShooterConstants.TowerZone.halfYSize)
                < ShooterConstants.TowerZone.halfYSize);
  }

  /** Returns if the shooter is in a spot where pass shots would be blocked by the hub. */
  @AutoLogOutput(key = "Shooter/IsBehindHub")
  public boolean isBehindHub() {
    final Translation2d shooterTranslation = getShooterFieldTranslation();
    final double absLocalY =
        Math.abs(shooterTranslation.getY() - FieldConstants.LinesHorizontal.center);
    final double localXNeutral =
        shooterTranslation.getX()
            - AllianceFlipUtil.applyX(FieldConstants.LinesVertical.neutralZoneNear);
    final double localXOpp =
        shooterTranslation.getX()
            - AllianceFlipUtil.applyX(FieldConstants.LinesVertical.oppAllianceZone);

    if (AllianceFlipUtil.shouldFlip()) {
      // Red alliance
      // Check general y position
      return absLocalY < ShooterConstants.BehindHubZone.halfBaseWidth
          // Check neutral zone position
          && ((Math.abs(localXNeutral + (ShooterConstants.BehindHubZone.halfHeight))
                      < ShooterConstants.BehindHubZone.halfHeight
                  && absLocalY
                      < ShooterConstants.BehindHubZone.halfBaseWidth
                          + (ShooterConstants.BehindHubZone.slope * localXNeutral))
              // Check opp alliance zone
              || (Math.abs(localXOpp + (ShooterConstants.BehindHubZone.halfHeight))
                      < ShooterConstants.BehindHubZone.halfHeight)
                  && absLocalY
                      < ShooterConstants.BehindHubZone.halfBaseWidth
                          + (ShooterConstants.BehindHubZone.slope * localXOpp));
    } else {
      // Blue alliance
      // Check general y position
      return absLocalY < ShooterConstants.BehindHubZone.halfBaseWidth
          // Check neutral zone position
          && ((Math.abs(localXNeutral - (ShooterConstants.BehindHubZone.halfHeight))
                      < ShooterConstants.BehindHubZone.halfHeight
                  && absLocalY
                      < ShooterConstants.BehindHubZone.halfBaseWidth
                          - (ShooterConstants.BehindHubZone.slope * localXNeutral))
              // Check opp alliance zone
              || (Math.abs(localXOpp - (ShooterConstants.BehindHubZone.halfHeight))
                      < ShooterConstants.BehindHubZone.halfHeight)
                  && absLocalY
                      < ShooterConstants.BehindHubZone.halfBaseWidth
                          - (ShooterConstants.BehindHubZone.slope * localXOpp));
    }
  }
}
