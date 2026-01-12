package frc.robot.commands;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds fieldRelativeSpeeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * DriveConstants.maxLinearVelocity,
                  linearVelocity.getY() * DriveConstants.maxLinearVelocity,
                  omega * DriveConstants.maxAngularVelocity);
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  fieldRelativeSpeeds,
                  AllianceFlipUtil.shouldFlip()
                      ? drive.getRotation().rotateBy(Rotation2d.kPi)
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle or controlling absolute rotation with a
   * joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Configure PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.angularPID.kP,
            DriveConstants.angularPID.kI,
            DriveConstants.angularPID.kD,
            new TrapezoidProfile.Constraints(
                DriveConstants.maxAngularVelocity, DriveConstants.maxAngularAcceleration));

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds fieldRelativeSpeeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * DriveConstants.maxLinearVelocity,
                      linearVelocity.getY() * DriveConstants.maxLinearVelocity,
                      omega);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      fieldRelativeSpeeds,
                      AllianceFlipUtil.shouldFlip()
                          ? drive.getRotation().rotateBy(Rotation2d.kPi)
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(
            () -> {
              angleController.enableContinuousInput(-Math.PI, Math.PI);
              angleController.reset(
                  drive.getRotation().getRadians(), drive.getChassisSpeeds().omegaRadiansPerSecond);
            });
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Similar to joystickDriveAtAngle but without the need to precalculate angle when pointing at a
   * single point.
   */
  public static Command joystickDriveAtTarget(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> targetSupplier) {

    // Configure PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.angularPID.kP,
            DriveConstants.angularPID.kI,
            DriveConstants.angularPID.kD,
            new TrapezoidProfile.Constraints(
                DriveConstants.maxAngularVelocity, DriveConstants.maxAngularAcceleration));

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(),
                      targetSupplier
                          .get()
                          .minus(drive.getPose().getTranslation())
                          .getAngle()
                          .getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds fieldRelativeSpeeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * DriveConstants.maxLinearVelocity,
                      linearVelocity.getY() * DriveConstants.maxLinearVelocity,
                      omega);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      fieldRelativeSpeeds,
                      AllianceFlipUtil.shouldFlip()
                          ? drive.getRotation().rotateBy(Rotation2d.kPi)
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(
            () -> {
              angleController.enableContinuousInput(-Math.PI, Math.PI);
              angleController.reset(
                  drive.getRotation().getRadians(), drive.getChassisSpeeds().omegaRadiansPerSecond);
            });
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Similar to joystickDriveAtAngle but without the need to precalculate angle when pointing at a
   * single point.
   */
  public static Command joystickDriveAtOptionalTarget(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Optional<Translation2d>> targetSupplier) {

    // Configure PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.angularPID.kP,
            DriveConstants.angularPID.kI,
            DriveConstants.angularPID.kD,
            new TrapezoidProfile.Constraints(
                DriveConstants.maxAngularVelocity, DriveConstants.maxAngularAcceleration));

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega;
              if (targetSupplier.get().isPresent()) {
                omega =
                    angleController.calculate(
                        drive.getRotation().getRadians(),
                        targetSupplier
                            .get()
                            .get()
                            .rotateAround(drive.getPose().getTranslation(), Rotation2d.kPi)
                            .minus(drive.getPose().getTranslation())
                            .getAngle()
                            .getRadians());
              } else {
                // Apply rotation deadband
                omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                // Square rotation value for more precise control
                omega = Math.copySign(omega * omega, omega) * DriveConstants.maxAngularVelocity;
              }

              // Convert to field relative speeds & send command
              ChassisSpeeds fieldRelativeSpeeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * DriveConstants.maxLinearVelocity,
                      linearVelocity.getY() * DriveConstants.maxLinearVelocity,
                      omega);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      fieldRelativeSpeeds,
                      AllianceFlipUtil.shouldFlip()
                          ? drive.getRotation().rotateBy(Rotation2d.kPi)
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(
            () -> {
              angleController.enableContinuousInput(-Math.PI, Math.PI);
              angleController.reset(
                  drive.getRotation().getRadians(), drive.getChassisSpeeds().omegaRadiansPerSecond);
            });
  }

  /**
   * Drive to a specified pose using autopilot. This command will run until the target pose it met.
   *
   * <ul>
   *   <li><b> APTarget controls:</b>
   *       <ul>
   *         <li><b> Reference </b> Target pose
   *         <li><b> EntryAngle </b> The entry angle of the robot
   *         <li><b> Velocity </b> The desired end velocity when the robot approaches the target
   *             (m/s)
   *             <ul>
   *               <li>If the target end velocity is greater then 0, <code> apConfigDynamic </code>
   *                   is used, otherwise <code> apConfigStatic </code> is used.
   *             </ul>
   *         <li><b> RotationRadius </b> The distance from the target pose that rotation goals are
   *             respected (meters)
   *             <ul>
   *               <li>By default, rotation goals are always respected. Adjusting this radius
   *                   prevents Autopilot from reorienting the robot until the robot is within the
   *                   specified radius of the target.
   *             </ul>
   *       </ul>
   * </ul>
   */
  public static Command autopilotDriveToPose(Drive drive, Supplier<APTarget> targetSupplier) {
    // Configure Autopilot controller
    Autopilot autopilot;
    if (targetSupplier.get().getVelocity() == 0.0) {
      autopilot = new Autopilot(DriveConstants.apConfigStatic);
    } else {
      autopilot = new Autopilot(DriveConstants.apConfigDynamic);
    }

    // Configure PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.angularPID.kP,
            DriveConstants.angularPID.kI,
            DriveConstants.angularPID.kD,
            new TrapezoidProfile.Constraints(
                DriveConstants.maxAngularVelocity, DriveConstants.maxAngularAcceleration));

    List<Pose2d> trajectory = new LinkedList<>();

    return Commands.run(
            () -> {
              // Logging
              trajectory.add(drive.getPose());
              Logger.recordOutput(
                  "Autopilot/Trajectory", trajectory.toArray(new Pose2d[trajectory.size()]));
              Logger.recordOutput(
                  "Autopilot/Target", new Pose2d[] {targetSupplier.get().getReference()});

              // Calculate Autopilot result
              APResult result =
                  autopilot.calculate(
                      drive.getPose(), drive.getChassisSpeeds(), targetSupplier.get());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), result.targetAngle().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds fieldRelativeSpeeds =
                  new ChassisSpeeds(
                      result.vx().baseUnitMagnitude(), result.vy().baseUnitMagnitude(), omega);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, drive.getRotation()));
            },
            drive)
        .repeatedly()

        // Before starting, configure angle controller, clear trajectory list, and logging
        .beforeStarting(
            () -> {
              angleController.enableContinuousInput(-Math.PI, Math.PI);
              angleController.reset(
                  drive.getRotation().getRadians(), drive.getChassisSpeeds().omegaRadiansPerSecond);

              trajectory.clear();

              Logger.recordOutput("Autopilot/State", "Moving to Target");
            })

        // Run until robot is within error of target pose
        .until(() -> autopilot.atTarget(drive.getPose(), targetSupplier.get()))

        // When at target or interupted, if end velocity is non-zero then stop the drive motors
        // Also reset logged values
        .finallyDo(
            () -> {
              if (targetSupplier.get().getVelocity() == 0.0) drive.stop();

              Logger.recordOutput("Autopilot/Trajectory", new Pose2d[] {});
              Logger.recordOutput("Autopilot/Target", new Pose2d[] {});

              if (autopilot.atTarget(drive.getPose(), targetSupplier.get())) {
                Logger.recordOutput("Autopilot/State", "At Target");
              } else {
                Logger.recordOutput("Autopilot/State", "Interrupted");
              }
            });
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
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
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
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
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
