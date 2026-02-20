package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
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

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  protected final FlyWheelIOInputsAutoLogged inputs = new FlyWheelIOInputsAutoLogged();
  private final Alert leaderDisconnectedAlert =
      new Alert("Flywheel leader (left) motor disconnected!", AlertType.kError);
  private final Alert followerDisconnectedAlert =
      new Alert("Flywheel follower (right) motor disconnected!", AlertType.kError);

  private final Alert leaderTempAlert =
      new Alert("Flywheel leader (left) motor is too hot.", AlertType.kWarning);
  private final Alert followerTempAlert =
      new Alert("Flywheel follower (right) motor is too hot.", AlertType.kWarning);

  private final Debouncer flywheelLeaderDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Debouncer flywheelFollowerDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private LoggedTunableNumber kP0 = new LoggedTunableNumber("Shooter/Flywheel/Slot0/kP", 20);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber("Shooter/Flywheel/Slot0/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber("Shooter/Flywheel/Slot0/kS", 0);

  private LoggedTunableNumber mmVelocity =
      new LoggedTunableNumber("Shooter/Flywheel/MotionMagic/Velocity", 0);
  private LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("Shooter/Flywheel/MotionMagic/Acceleration", 0);
  private LoggedTunableNumber mmJerk =
      new LoggedTunableNumber("Shooter/Flywheel/MotionMagic/Jerk", 0);
  private LoggedTunableNumber setpointBandVelocity =
      new LoggedTunableNumber("Shooter/Flywheel/VelocitySetpointBandPercent", 0.1);

  @Getter private double setpoint = 0.0;

  @Getter private ControlMode mode = ControlMode.Neutral;

  public Flywheel(FlywheelIO flywheelIO) {
    this.io = flywheelIO;
  }

  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel", inputs);

    // Update alerts
    leaderDisconnectedAlert.set(!flywheelLeaderDebouncer.calculate(inputs.leaderConnected));
    followerDisconnectedAlert.set(!flywheelFollowerDebouncer.calculate(inputs.followerConnected));
    leaderTempAlert.set(inputs.leaderTempCelsius > Constants.warningTempCelsius);
    followerTempAlert.set(inputs.followerTempCelsius > Constants.warningTempCelsius);

    // Update logged setpoints
    Logger.recordOutput(
        "Shooter/Flywheel/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput(
        "Shooter/Flywheel/SetpointVelocityRotsPerSec",
        (mode == ControlMode.Velocity) ? setpoint : 0);

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
   * Run system to specified velocity.
   *
   * <p><b>Units:</b> Mechanism rotations per second.
   *
   * @param velocity Goal velocity.
   * @param slot PID gain slot to use during motion.
   */
  public void runVelocity(double velocity, int slot) {
    setpoint = velocity;
    mode = ControlMode.Velocity;
    // TODO: use slot provided instead of always using slot 0
    io.runVelocity(velocity, 0);
  }

  /** Stop motor with neutral output. */
  public void stop() {
    mode = ControlMode.Neutral;
    io.stop();
  }

  /**
   * Velocity of the system in mechanism rotations per second.
   *
   * @return Velocity.
   */
  public double getVelocity() {
    return inputs.velocityRotsPerSec;
  }

  /**
   * Current corresponding to the torque output by the motor. Similar to StatorCurrent. Users will
   * likely prefer this current to calculate the applied torque to the rotor.
   *
   * <p>Stator current where positive current means torque is applied in the forward direction as
   * determined by the Inverted setting.
   *
   * @return Lead motor torque current.
   */
  public double getTorqueCurrent() {
    return inputs.leaderTorqueCurrentAmps;
  }

  /**
   * Returns true if the percent error is within the tolerance of the setpoint.
   *
   * <p>This will return false when not velocity controlled.
   *
   * @return Whether the percent error is within the acceptable bounds.
   */
  @AutoLogOutput(key = "Shooter/Flywheel/atSetpoint")
  public boolean atSetpoint() {
    return switch (mode) {
      case Velocity ->
          (getVelocity() == 0.0)
              // TODO: this can cause a bug if setpoint is zero, but we shouldn't encounter it in a
              // match.
              // TODO: decide if this should actually be based off percent.
              ? false
              : Math.abs((setpoint / getVelocity()) - 1) < setpointBandVelocity.get();
      default -> false;
    };
  }
}
