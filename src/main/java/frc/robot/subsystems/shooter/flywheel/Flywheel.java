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

  // TODO: We should change this to be based off percent error
  // That means we should change the name to reflect
  // and we also need to set this to a number 10% should be good to start
  private LoggedTunableNumber setpointBandVelocity =
      new LoggedTunableNumber("Shooter/Flywheel/VelocitySetpointBand", 0);

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
   * Set applied voltage to the motor
   *
   * @param inputVolts Voltage to drive motor at
   */
  public void runVolts(double volts) {
    setpoint = volts;
    mode = ControlMode.Voltage;

    io.runVolts(volts);
  }

  public void runVelocity(double velocity, int slot) {
    setpoint = velocity;
    mode = ControlMode.Velocity;
    io.runVelocity(velocity, 0);
  }

  /**
   * Set goal position in mechanism rotations
   *
   * @param position Goal position
   */

  /** Stop motor */
  public void stop() {
    mode = ControlMode.Neutral;
    io.stop();
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

  /**
   * Check if mechanism is at goal position with error of setpointBandPosition
   *
   * @return True if in position control mode and mechanism is at goal position, false otherwise
   */
  @AutoLogOutput(key = "Shooter/Flywheel/atSetpoint")
  public boolean atSetpoint() {
    return switch (mode) {
        // TODO: update this to relect the change to percent error
      case Velocity -> Math.abs(setpoint - getVelocity()) < setpointBandVelocity.get();
      default -> false;
    };
  }
}
