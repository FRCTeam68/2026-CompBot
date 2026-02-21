package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// TODO: add logic if we turn the robot on too close to the limits throw error and don't run
public class Turret extends SubsystemBase {
  @Getter private static final double minimum = 0;
  @Getter private static final double maximum = 360;

  private final TurretIO io;
  protected final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Turret motor disconnected!", AlertType.kError);
  private final Alert turretCancoderDisconnectedAlert =
      new Alert("Turret cancoder disconnected!", AlertType.kError);
  private final Alert motorTempAlert = new Alert("Turret motor is too hot.", AlertType.kWarning);
  private final Debouncer motorDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Debouncer cancoderDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private LoggedTunableNumber kP0 = new LoggedTunableNumber("Shooter/Turret/Slot0/kP", 100);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber("Shooter/Turret/Slot0/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber("Shooter/Turret/Slot0/kS", 0);

  private LoggedTunableNumber mmVelocity =
      new LoggedTunableNumber("Shooter/Turret/MotionMagic/Velocity", 0);
  private LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("Shooter/Turret/MotionMagic/Acceleration", 0);
  private LoggedTunableNumber mmJerk = new LoggedTunableNumber("Shooter/Turret/MotionMagic/Jerk", 0);

  private LoggedTunableNumber setpointBandPosition =
      new LoggedTunableNumber("Shooter/Turret/PositionSetpointBand", 10);

  @Getter private double setpoint = 0.0;

  @Getter private ControlMode mode = ControlMode.Neutral;

  public Turret(TurretIO io) {
    this.io = io;
  }

  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret", inputs);

    // Update alerts
    motorDisconnectedAlert.set(!motorDebouncer.calculate(inputs.motorConnected));
    turretCancoderDisconnectedAlert.set(!cancoderDebouncer.calculate(inputs.cancoderConnected));
    motorTempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);

    // Update logged setpoints
    Logger.recordOutput(
        "Shooter/Turret/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput(
        "Shooter/Turret/SetpointPositionDeg", (mode == ControlMode.Position) ? setpoint : 0);

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
   * Run system to position.
   *
   * <p><b>Units:</b> Mechanism degrees.
   *
   * @param degrees Goal position.
   * @param slot PID gain slot to use during motion.
   */
  public void runPosition(double degrees, int slot) {
    setpoint = MathUtil.inputModulus(degrees, minimum, maximum);
    mode = ControlMode.Position;
    io.runPosition(setpoint, 0);
  }

  /** Stop motor with neutral output. */
  public void stop() {
    mode = ControlMode.Neutral;
    io.stop();
  }

  /** Set the current mechanism position to zero. */
  public void zero() {
    io.setPosition(0);
  }

  // TODO: delete this whole method. We only need to zero at the zero position.
  public void setPosition(double rotations) {
    io.setPosition(rotations);
  }

  // TODO: create a new method to set the current position. This is part of the system to make sure
  // we don't overrotate by booting in the wrong location. Since the encoder is 1:1 with the turret
  // it will always have the correct angle, but it may be greater than 1 full rotation. This method
  // should set the position to the current position mod 360 degrees.

  /**
   * Position of the system in mechanism degrees.
   *
   * @return Position.
   */
  public double getPosition() {
    return inputs.positionDeg;
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
  @AutoLogOutput(key = "Shooter/Turret/atSetpoint")
  public boolean atSetpoint() {
    return switch (mode) {
      case Position -> Math.abs(setpoint - getPosition()) < setpointBandPosition.get();
      default -> false;
    };
  }
}
