package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// TODO: the flywheel class needs to extend SubsystemBase to run the periodic method.
public class Flywheel {
  private final FlywheelIO io;
  protected final FlyWheelIOInputsAutoLogged inputs = new FlyWheelIOInputsAutoLogged();
  private final Alert flywheelLeaderDisconnectedAlert =
      new Alert("Flywheel(left) disconnected!", AlertType.kError);
  private final Alert flywheelFollowerDisconnectedAlert =
      new Alert("Flywheel(right) disconnected!", AlertType.kError);

  private final Alert flywheelLeaderTempAlert =
      new Alert("Flywheel(left) is too hot.", AlertType.kWarning);
  private final Alert flywheelFollowerTempAlert =
      new Alert("Flywheel(right) is too hot.", AlertType.kWarning);

  private final Debouncer flywheelLeaderDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Debouncer flywheelFollowerDebouncer = new Debouncer(0.5, DebounceType.kRising);

  // TODO: This is still logging to the MotorTemplate folder
  private LoggedTunableNumber kP0 = new LoggedTunableNumber("MotorTemplate/Slot0/kP", 0);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber("MotorTemplate/Slot0/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber("MotorTemplate/Slot0/kS", 0);

  // TODO: This is still logging to the MotorTemplate folder
  private LoggedTunableNumber mmVelocity =
      new LoggedTunableNumber("MotorTemplate/MotionMagic/Velocity", 0);
  private LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("MotorTemplate/MotionMagic/Acceleration", 0);
  private LoggedTunableNumber mmJerk = new LoggedTunableNumber("MotorTemplate/MotionMagic/Jerk", 0);

  // TODO: This is still logging to the MotorTemplate folder
  private LoggedTunableNumber setpointBandVelocity =
      new LoggedTunableNumber("MotorTemplate/VelocitySetpointBand", 0);

  @Getter private double setpoint = 0.0;

  @Getter private ControlMode mode = ControlMode.Neutral;

  public Flywheel(FlywheelIO flywheelIO) {
    this.io = flywheelIO;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    flywheelLeaderDisconnectedAlert.set(!flywheelLeaderDebouncer.calculate(inputs.leaderConnected));
    flywheelFollowerDisconnectedAlert.set(
        !flywheelFollowerDebouncer.calculate(inputs.followerConnected));
    flywheelLeaderTempAlert.set(inputs.leaderTempCelsius > Constants.warningTempCelsius);
    flywheelFollowerTempAlert.set(inputs.followerTempCelsius > Constants.warningTempCelsius);

    // TODO: This is still logging to the MotorTemplate folder
    Logger.recordOutput(
        "MotorTemplate/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput(
        "MotorTemplate/SetpointVelocityRotsPerSec", (mode == ControlMode.Velocity) ? setpoint : 0);

    // Update tunable numbers
    if (kP0.hasChanged(hashCode()) || kD0.hasChanged(hashCode()) || kS0.hasChanged(hashCode())) {
      // TODO: uncomment this
      // io.setPID(new SlotConfigs().withKP(kP0.get()).withKD(kD0.get()).withKS(kS0.get()));
    }

    if (mmVelocity.hasChanged(hashCode())
        || mmAcceleration.hasChanged(hashCode())
        || mmJerk.hasChanged(hashCode())) {
      // TODO: uncomment this
      // io.setMotionMagic(
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(mmVelocity.get())
          .withMotionMagicAcceleration(mmAcceleration.get())
          .withMotionMagicJerk(mmJerk.get());
    }
  }

  /**
   * Set applied voltage to the motor
   *
   * @param inputVolts Voltage to drive motor at
   */
  public void runVolts(double volts) {
    setpoint = volts;
    mode = ControlMode.Voltage;
    // TODO: uncomment this
    // io.runVolts(volts);
  }

  public void runVelocity(double velocity, int slot) {
    mode = ControlMode.Velocity;
    // TODO: uncomment this
    // io.runVelocity(velocity, 0);
  }

  /**
   * Set goal position in mechanism rotations
   *
   * @param position Goal position
   */

  /** Stop motor */
  public void stop() {
    mode = ControlMode.Neutral;
    // TODO: uncomment this
    // io.stop();
  }

  /** Set the current mechanism position to zero */
  public void zero() {
    // TODO: uncomment this
    // io.setPosition(0);
  }

  /**
   * Velocity of the mechanism in degrees of elevation per second
   *
   * @return Velocity
   */
  public double getVelocity() {
    return inputs.velocityRotsPerSec;
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
    return inputs.leaderTorqueCurrentAmps;
  }

  // TODO: This is still logging to the MotorTemplate folder
  /**
   * Check if mechanism is at goal position with error of setpointBandPosition
   *
   * @return True if in position control mode and mechanism is at goal position, false otherwise
   */
  @AutoLogOutput(key = "MotorTemplate/atSetpoint")
  public boolean atSetpoint() {
    return switch (mode) {
      case Velocity -> Math.abs(setpoint - inputs.velocityRotsPerSec) < setpointBandVelocity.get();
      default -> false;
    };
  }
}
