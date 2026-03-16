package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotSystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.LoggedTunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class TestCommands {
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final Drive drive = robotSystem.getDrive();
  private static final IntakePivot intakePivot = robotSystem.getIntakePivot();
  private static final Shooter shooter = robotSystem.getShooter();

  public static Command functionTest() {
    // Drive forward/backward
    // Rotate drive wheels counterclockwise
    // Deploy intake
    // Retract intake
    // Deploy intake
    // Agitate intake
    // Deploy intake
    // Spin intake
    // Stop intake spin
    // Run turret to limits
    // Run hood to limits
    // Run shooter to max speed
    // Run shooter at fountain config
    // Run spindexer/feeder
    // Stop all
    // Retract intake
    // Move hood to minimum elevation
    // Move turret to 180
    AtomicReference<Double> driveTurnOffest = new AtomicReference<Double>(0.0);
    return Commands.sequence(
        ShooterCommands.runStatic(
            0, shooter.getHood().getElevation(), shooter.getTurret().getPosition()),
        DriveCommands.joystickDrive(() -> -1, () -> 0, () -> 0).withTimeout(1),
        DriveCommands.joystickDrive(() -> 1, () -> 0, () -> 0).withTimeout(1),
        Commands.runOnce(() -> driveTurnOffest.set(Timer.getTimestamp())),
        DriveCommands.joystickDrive(
                () -> Math.cos(Timer.getTimestamp() - driveTurnOffest.get()),
                () -> Math.sin(Timer.getTimestamp() - driveTurnOffest.get()),
                () -> 0)
            .until(() -> Timer.getTimestamp() > driveTurnOffest.get() + (2 * Math.PI)),
        Commands.deadline(
            Commands.sequence(
                IntakeCommands.intakeAutomatic().withTimeout(0.1),
                Commands.waitUntil(() -> intakePivot.atSetpoint()),
                Commands.waitSeconds(0.5),
                IntakeCommands.retract().withTimeout(2),
                IntakeCommands.intakeAutomatic().withTimeout(0.1),
                Commands.waitUntil(() -> intakePivot.atSetpoint()),
                Commands.waitSeconds(0.5),
                IntakeCommands.agitate(2),
                IntakeCommands.intakeAutomatic().withTimeout(3),
                Commands.runOnce(() -> shooter.getTurret().runPosition(0.01), shooter),
                Commands.waitUntil(() -> shooter.getTurret().atSetpoint()),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> shooter.getTurret().runPosition(0), shooter),
                Commands.waitUntil(() -> shooter.getTurret().atSetpoint()),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> shooter.getHood().runElvation(Hood.getMinimum()), shooter),
                Commands.waitUntil(() -> shooter.getHood().atSetpoint()),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> shooter.getHood().runElvation(Hood.getMaximum()), shooter),
                Commands.waitUntil(() -> shooter.getHood().atSetpoint()),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> shooter.getFlywheel().runVelocity(70), shooter),
                Commands.waitUntil(() -> shooter.getHood().atSetpoint()),
                Commands.waitSeconds(0.5),
                ShooterCommands.runStatic(ShooterConstants.StaticShot.fountain),
                Commands.waitUntil(() -> shooter.atSetpoint()),
                ShooterCommands.shoot(false).withTimeout(5),
                ShooterCommands.runStatic(0, Hood.getMaximum(), 180),
                IntakeCommands.retract()),
            DriveCommands.joystickDrive(() -> 0, () -> 0, () -> 0)));
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Similar to joystickDriveAtAngle but without the need to precalculate angle when pointing at a
   * single point.
   */
  public static Command driveTuning(Pose2d targetPose) {
    // Configure tunable numbers
    LoggedTunableNumber linearKP = new LoggedTunableNumber("Drive/Tuning/LinearKP", 0);
    LoggedTunableNumber linearKD = new LoggedTunableNumber("Drive/Tuning/LinearKD", 0);
    LoggedTunableNumber angularKP = new LoggedTunableNumber("Drive/Tuning/AngularKP", 0);
    LoggedTunableNumber angularKD = new LoggedTunableNumber("Drive/Tuning/AngularKD", 0);

    // Configure PID controllers
    AtomicReference<ProfiledPIDController> xController = new AtomicReference<>();
    AtomicReference<ProfiledPIDController> yController = new AtomicReference<>();
    AtomicReference<ProfiledPIDController> angleController = new AtomicReference<>();

    // Construct command
    return Commands.run(
            () -> {
              // // Update constraints
              // angleController.get().setConstraints(
              //     new TrapezoidProfile.Constraints(
              //         getAdjustedMaxAngularVelocity(), DriveConstants.maxAngularAcceleration));

              // // Calculate linear speed

              // // Calculate angular speed
              // double omega =
              //     angleController.get().calculate(
              //         drive.getRotation().getRadians(),
              //         targetPose.getTranslation()
              //             .minus(drive.getPose().getTranslation())
              //             .getAngle()
              //             .getRadians());

              // // Convert to field relative speeds & send command
              // ChassisSpeeds fieldRelativeSpeeds =
              //     new ChassisSpeeds(
              //         omega);
              // if (robotSystem.isShooting && isChassisSpeedsZero(fieldRelativeSpeeds)) {
              //   drive.stopWithX();
              // } else {
              //   drive.runVelocity(
              //       ChassisSpeeds.fromFieldRelativeSpeeds(
              //           fieldRelativeSpeeds,
              //           AllianceFlipUtil.shouldFlip()
              //               ? drive.getRotation().rotateBy(Rotation2d.kPi)
              //               : drive.getRotation()));
              // }
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(
            () -> {
              angleController.set(
                  new ProfiledPIDController(
                      DriveConstants.angularPID.kP,
                      0.0,
                      DriveConstants.angularPID.kD,
                      new TrapezoidProfile.Constraints(
                          DriveConstants.maxAngularVelocity,
                          DriveConstants.maxAngularAcceleration)));
              angleController.get().enableContinuousInput(-Math.PI, Math.PI);
              angleController
                  .get()
                  .reset(
                      drive.getRotation().getRadians(),
                      drive.getChassisSpeeds().omegaRadiansPerSecond);

              xController.set(
                  new ProfiledPIDController(
                      DriveConstants.angularPID.kP,
                      0.0,
                      DriveConstants.angularPID.kD,
                      new TrapezoidProfile.Constraints(
                          DriveConstants.maxAngularVelocity,
                          DriveConstants.maxAngularAcceleration)));
              xController
                  .get()
                  .reset(drive.getPose().getX(), drive.getChassisSpeeds().vxMetersPerSecond);

              yController.set(
                  new ProfiledPIDController(
                      DriveConstants.angularPID.kP,
                      0.0,
                      DriveConstants.angularPID.kD,
                      new TrapezoidProfile.Constraints(
                          DriveConstants.maxAngularVelocity,
                          DriveConstants.maxAngularAcceleration)));
              yController
                  .get()
                  .reset(drive.getPose().getY(), drive.getChassisSpeeds().vyMetersPerSecond);
            })
        .withName("DriveTuning");
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization() {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                  velocitySamples.clear();
                  voltageSamples.clear();
                }),

            // Allow modules to orient
            Commands.run(
                    () -> {
                      drive.runCharacterization(0.0);
                    },
                    drive)
                .withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                    () -> {
                      double voltage = timer.get() * FF_RAMP_RATE;
                      drive.runCharacterization(voltage);
                      velocitySamples.add(drive.getFFCharacterizationVelocity());
                      voltageSamples.add(voltage);
                    },
                    drive)

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      int n = velocitySamples.size();
                      double sumX = 0.0;
                      double sumY = 0.0;
                      double sumXY = 0.0;
                      double sumX2 = 0.0;
                      for (int i = 0; i < n; i++) {
                        sumX += velocitySamples.get(i);
                        sumY += voltageSamples.get(i);
                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                      }
                      double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                      double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                      NumberFormat formatter = new DecimalFormat("#0.00000");
                      System.out.println("********** Drive FF Characterization Results **********");
                      System.out.println("\tkS: " + formatter.format(kS));
                      System.out.println("\tkV: " + formatter.format(kV));
                    }))
        .withName("DriveFeedforwardCharacterization");
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization() {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(
                    () -> {
                      limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                      double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                      state.positions = drive.getWheelRadiusCharacterizationPositions();
                      state.lastAngle = drive.getRotation();
                      state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                        () -> {
                          var rotation = drive.getRotation();
                          state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                          state.lastAngle = rotation;
                        })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                          double[] positions = drive.getWheelRadiusCharacterizationPositions();
                          double wheelDelta = 0.0;
                          for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                          }
                          double wheelRadius =
                              (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                          NumberFormat formatter = new DecimalFormat("#0.000");
                          System.out.println(
                              "********** Wheel Radius Characterization Results **********");
                          System.out.println(
                              "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                          System.out.println(
                              "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                          System.out.println(
                              "\tWheel Radius: "
                                  + formatter.format(wheelRadius)
                                  + " meters, "
                                  + formatter.format(Units.metersToInches(wheelRadius))
                                  + " inches");
                        })))
        .withName("DriveWheelRadiusCharacterization");
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
