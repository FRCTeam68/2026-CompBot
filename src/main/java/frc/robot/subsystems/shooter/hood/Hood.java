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

  private LoggedTunableNumber kP0 = new LoggedTunableNumber("Shooter/Hood/Slot0/kP", 20);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber("Shooter/Hood/Slot0/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber("Shooter/Hood/Slot0/kS", 0);

  private LoggedTunableNumber mmVelocity =
      new LoggedTunableNumber("Shooter/Hood/MotionMagic/Velocity", 0);
  private LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("Shooter/Hood/MotionMagic/Acceleration", 0);
  private LoggedTunableNumber mmJerk = new LoggedTunableNumber("Shooter/Hood/MotionMagic/Jerk", 0);
  private LoggedTunableNumber setpointBandPosition =
      new LoggedTunableNumber("Shooter/Hood/PositionSetpointBand", 2);

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
    Logger.processInputs("Shooter/Hood", inputs);

    // Update alerts
    motorDisconnectedAlert.set(!motorDebouncer.calculate(inputs.motorConnected));
    hoodCancoderDisconnectedAlert.set(!cancoderDebouncer.calculate(inputs.cancoderConnected));
    motorTempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);

    // Update logged setpoints
    Logger.recordOutput("Shooter/Hood/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput(
        "Shooter/Hood/SetpointPositionRots", (mode == ControlMode.Position) ? setpoint : 0);

    // Lower hood if in tench box
    if (prevInTrenchBox != inTrenchBox()) {
      if (getElevation() < underTrenchMinimum || setpoint < underTrenchMinimum) {
        runElvation(setpoint, 0);
      }
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
    setpoint = MathUtil.clamp(elevation, minimum, maximum);
    mode = ControlMode.Position;
    io.runPosition(
        (inTrenchBox()) ? MathUtil.clamp(setpoint, underTrenchMinimum, maximum) : setpoint, slot);
  }

  /** Stop motor with neutral output. */
  public void stop() {
    mode = ControlMode.Neutral;
    io.stop();
  }

  /** Set the current mechanism position to the maximum. */
  public void setPositionMaximum() {
    io.setPosition(maximum);
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
  @AutoLogOutput(key = "Shooter/Hood/atSetpoint")
  public boolean atSetpoint() {
    return switch (mode) {
      case Position -> Math.abs(setpoint - getElevation()) < setpointBandPosition.get();
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
