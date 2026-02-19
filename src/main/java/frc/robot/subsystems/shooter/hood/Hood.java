package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// TODO: ** lets log everything inside of the shooter folder
public class Hood extends SubsystemBase {
  @Getter private static final double minimum = 53.368453;
  @Getter private static final double maximum = 79.368453;
  @Getter private static final double underTrenchMinimum = maximum - 9;

  private final Supplier<Pose2d> poseSupplier;
  private final HoodIO io;
  protected final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Hood motor disconnected!", AlertType.kError);
  private final Alert hoodCancoderDisconnectedAlert =
      new Alert("Hood cancoder disconnected!", AlertType.kError);
  private final Alert motorTempAlert = new Alert("Hood motor is too hot.", AlertType.kWarning);
  private final Debouncer motorDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Debouncer cancoderDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private LoggedTunableNumber kP0 = new LoggedTunableNumber("Hood/Slot0/kP", 20);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber("Hood/Slot0/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber("Hood/Slot0/kS", 0);

  private LoggedTunableNumber mmVelocity = new LoggedTunableNumber("Hood/MotionMagic/Velocity", 0);
  private LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("Hood/MotionMagic/Acceleration", 0);
  private LoggedTunableNumber mmJerk = new LoggedTunableNumber("Hood/MotionMagic/Jerk", 0);
  private LoggedTunableNumber setpointBandPosition =
      new LoggedTunableNumber("Hood/PositionSetpointBand", 2);

  @Getter private double setpoint = 0.0;
  @Getter private ControlMode mode = ControlMode.Neutral;
  private boolean prevInTrenchBox = false;

  public Hood(HoodIO hoodIO, Supplier<Pose2d> poseSupplier) {
    this.io = hoodIO;
    this.poseSupplier = poseSupplier;
  }

  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // Update alerts
    motorDisconnectedAlert.set(!motorDebouncer.calculate(inputs.motorConnected));
    hoodCancoderDisconnectedAlert.set(!cancoderDebouncer.calculate(inputs.cancoderConnected));
    motorTempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);

    // Update logged setpoints
    Logger.recordOutput("Hood/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput("Hood/SetpointPositionRots", (mode == ControlMode.Position) ? setpoint : 0);

    // Lower hood if in tench box
    if (prevInTrenchBox != inTrenchBox()) {
      // TODO: add an if statment to only run this if necessary i.e. the current position is too
      // high or the setpoint is putting it too high
      runElvation(setpoint, 0);
      prevInTrenchBox = inTrenchBox();
    }
    // Update tunable numbers
    if (kP0.hasChanged(hashCode()) | kD0.hasChanged(hashCode()) | kS0.hasChanged(hashCode())) {
      io.setPID(new SlotConfigs().withKP(kP0.get()).withKD(kD0.get()).withKS(kS0.get()));
    }

    if (mmVelocity.hasChanged(hashCode())
        | mmAcceleration.hasChanged(hashCode())
        | mmJerk.hasChanged(hashCode())) {
      io.setMotionMagic(
          new MotionMagicConfigs()
              .withMotionMagicCruiseVelocity(mmVelocity.get())
              .withMotionMagicAcceleration(mmAcceleration.get())
              .withMotionMagicJerk(mmJerk.get()));
    }
  }

  /**
   * Run system at specified voltage.
   *
   * @param volts Voltage to run the motor at.
   */
  public void runVolts(double volts) {
    setpoint = volts;
    mode = ControlMode.Voltage;
    io.runVolts(volts);
  }

  /**
   * Run system to specified elevation.
   *
   * <p><b>Units:</b> Mechanism degrees.
   *
   * @param elevation Goal elevation.
   * @param slot PID gain slot to use during motion.
   */
  public void runElvation(double elevation, int slot) {
    // TODO: clamp position to be within the limits. Then for the rest of this method use the
    // clamped setpoint.
    setpoint = elevation;
    mode = ControlMode.Position;
    io.runPosition(
        // TODO: change this to use the new variable underTrenchMinimum
        (inTrenchBox()) ? MathUtil.clamp(elevation, maximum - 9, maximum) : elevation, slot);
  }

  /** Stop motor with neutral output. */
  public void stop() {
    mode = ControlMode.Neutral;
    io.stop();
  }

  /** Set the current mechanism position to the maximum. */
  // TODO: change the name of this method to setPositionMaximum to relect its new function
  public void zero() {
    // TODO: change the zeroing position to the maximum. That is where the hard limit is.
    io.setPosition(0);
  }

  // TODO: delete this whole method. We only need to zero at the max position.
  public void setElvation(double degrees) {
    io.setPosition(degrees);
  }

  /**
   * Elevation of the system in mechanism degrees.
   *
   * @return Elevation.
   */
  public double getElevation() {
    return inputs.positionElvation;
  }

  /**
   * Current corresponding to the torque output by the motor. Similar to StatorCurrent. Users will
   * likely prefer this current to calculate the applied torque to the rotor.
   *
   * <p>Stator current where positive current means torque is applied in the forward direction as
   * determined by the Inverted setting.
   *
   * @return Motor torque current.
   */
  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  /**
   * Returns true if the error is within the tolerance of the setpoint.
   *
   * <p>This will return false when not position controlled.
   *
   * @return Whether the error is within the acceptable bounds.
   */
  @AutoLogOutput(key = "Hood/atSetpoint")
  public boolean atSetpoint() {
    return switch (mode) {
      case Position -> Math.abs(setpoint - inputs.positionElvation) < setpointBandPosition.get();
      default -> false;
    };
  }

  /**
   * Checks if the shooter is near any of the trenches. If so, the hood should be forced down to
   * avoid collisions.
   *
   * @return If the shooter is near the trench.
   */
  @AutoLogOutput(key = "Shooter/Hood/InTrenchBox")
  public boolean inTrenchBox() {
    Pose2d shooterPosistion =
        new Pose2d(ShooterConstants.shooterPosition.toTranslation2d(), Rotation2d.kZero)
            .plus(
                new Transform2d(
                    poseSupplier.get().getTranslation(), poseSupplier.get().getRotation()))
            .rotateAround(poseSupplier.get().getTranslation(), poseSupplier.get().getRotation());
    double xSize = Units.inchesToMeters(47);
    if (shooterPosistion.getY() < FieldConstants.LinesHorizontal.rightTrenchOpenStart
        || shooterPosistion.getY() > FieldConstants.LinesHorizontal.leftTrenchOpenEnd) {
      if ((shooterPosistion.getX() > FieldConstants.LinesVertical.hubCenter - (xSize / 2)
              && shooterPosistion.getX() < FieldConstants.LinesVertical.hubCenter + (xSize / 2))
          || (shooterPosistion.getX() > FieldConstants.LinesVertical.oppHubCenter - (xSize / 2)
              && shooterPosistion.getX()
                  < FieldConstants.LinesVertical.oppHubCenter + (xSize / 2))) {
        return true;
      }
    }
    return false;
  }
}
