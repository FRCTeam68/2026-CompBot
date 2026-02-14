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
  private final Supplier<Pose2d> poseSupplier;
  private final HoodIO io;
  protected final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final Alert hoodMotorDisconnectedAlert =
      new Alert("Hood motor disconnected!", AlertType.kError);
  private final Alert hoodCancoderDisconnectedAlert =
      new Alert("Hood cancoder disconnected!", AlertType.kError);

  private final Alert hoodMotorTempAlert = new Alert("Hood motor is too hot.", AlertType.kWarning);
  private final Alert hoodCancoderTempAlert =
      new Alert("Hood cancoder is too hot.", AlertType.kWarning);
  private final Debouncer hoodMotorDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Debouncer hoodCancoderDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private LoggedTunableNumber kP0 = new LoggedTunableNumber("Hood/Slot0/kP", 0);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber("Hood/Slot0/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber("Hood/Slot0/kS", 0);

  private LoggedTunableNumber mmVelocity = new LoggedTunableNumber("Hood/MotionMagic/Velocity", 0);
  private LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("Hood/MotionMagic/Acceleration", 0);
  private LoggedTunableNumber mmJerk = new LoggedTunableNumber("Hood/MotionMagic/Jerk", 0);

  private LoggedTunableNumber setpointBandPosition =
      new LoggedTunableNumber("Hood/PositionSetpointBand", 0);

  @Getter private double setpoint = 0.0;
  private boolean prevInTrenchBox = inTrenchBox();
  @Getter private ControlMode mode = ControlMode.Neutral;

  public Hood(HoodIO hoodIO, Supplier<Pose2d> poseSupplier) {
    this.io = hoodIO;
    this.poseSupplier = poseSupplier;
  }

  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    hoodMotorDisconnectedAlert.set(!hoodMotorDebouncer.calculate(inputs.motorConnected));
    hoodCancoderDisconnectedAlert.set(!hoodCancoderDebouncer.calculate(inputs.cancoderConnected));
    hoodMotorTempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);
    hoodCancoderTempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);

    Logger.recordOutput("Hood/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput("Hood/SetpointPositionRots", (mode == ControlMode.Position) ? setpoint : 0);

    if (prevInTrenchBox != inTrenchBox() && mode == ControlMode.Position) {
      runElvation(setpoint, 0);
      prevInTrenchBox = inTrenchBox();
    }

    // Update tunable numbers
    if (kP0.hasChanged(hashCode()) || kD0.hasChanged(hashCode()) || kS0.hasChanged(hashCode())) {
      io.setPID(new SlotConfigs().withKP(kP0.get()).withKD(kD0.get()).withKS(kS0.get()));
    }

    if (mmVelocity.hasChanged(hashCode())
        || mmAcceleration.hasChanged(hashCode())
        || mmJerk.hasChanged(hashCode())) {
      io.setMotionMagic(
          new MotionMagicConfigs()
              .withMotionMagicCruiseVelocity(mmVelocity.get())
              .withMotionMagicAcceleration(mmAcceleration.get())
              .withMotionMagicJerk(mmJerk.get()));
    }
  }

  public void setAtSetpointBandPosition(LoggedTunableNumber band) {
    setpointBandPosition = band;
  }

  /**
   * Set applied voltage to the motor
   *
   * @param inputVolts Voltage to drive motor at
   */
  public void runVolts(double volts) {
    setpoint = volts;
    mode = ControlMode.Voltage;
    io.runVolts(volts);
  }

  /**
   * Set goal position in mechanism rotations
   *
   * @param position Goal position
   */
  public void runElvation(double position, int slot) {
    mode = ControlMode.Position;
    io.runPosition(
        (inTrenchBox()) ? MathUtil.clamp(position, maximum - 9, maximum) : position, slot);
    setpoint = position;
  }

  /** Stop motor */
  public void stop() {
    mode = ControlMode.Neutral;
    io.stop();
  }

  /** Set the current mechanism position to zero */
  public void zero() {
    io.setPosition(0);
  }

  /**
   * Set the current mechanism position
   *
   * @param rotations Position in mechanism rotations
   */
  public void setElvation(double degrees) {
    io.setPosition(degrees);
  }

  /**
   * Position of the mechanism in degrees of elevation
   *
   * @return Elevation of the wrist
   */
  public double getPosition() {
    return inputs.positionElvation;
  }

  /**
   * Current corresponding to the torque output by the lead motor. Similar to StatorCurrent. Users
   * will likely prefer this current to calculate the applied torque to the rotor.
   *
   * <p>Stator current where positive current means torque is applied in the forward direction as
   * determined by the Inverted setting.
   *
   * @return Lead motor torque current
   */
  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  /**
   * Check if mechanism is at goal position with error of setpointBandPosition
   *
   * @return True if in position control mode and mechanism is at goal position, false otherwise
   */
  @AutoLogOutput(key = "Hood/atSetpoint")
  public boolean atSetpoint() {
    return switch (mode) {
      case Position -> Math.abs(setpoint - inputs.positionElvation) < setpointBandPosition.get();
      default -> false;
    };
  }

  @AutoLogOutput(key = "Shooter/InTrenchBox")
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
