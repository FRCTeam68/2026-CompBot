package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  // PID gains
  private LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Flywheel/kP", 3.0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Flywheel/kD", 0);
  private LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Flywheel/kS", 2.5);

  // Setpoint band
  private LoggedTunableNumber setpointBandVelocity =
      new LoggedTunableNumber("Shooter/Flywheel/VelocitySetpointBandPercent", 0.3);

  // Alerts
  private final Alert leaderDisconnectedAlert =
      new Alert("Flywheel leader (left) motor disconnected!", AlertType.kError);
  private final Alert followerDisconnectedAlert =
      new Alert("Flywheel follower (right) motor disconnected!", AlertType.kError);
  private final Alert leaderTempAlert =
      new Alert("Flywheel leader (left) motor is too hot.", AlertType.kWarning);
  private final Alert followerTempAlert =
      new Alert("Flywheel follower (right) motor is too hot.", AlertType.kWarning);

  // Debouncers
  private final Debouncer leaderConnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private final FlywheelIO io;
  protected final FlyWheelIOInputsAutoLogged inputs = new FlyWheelIOInputsAutoLogged();
  @Getter private double setpoint = 0.0;
  @Getter private ControlMode mode = ControlMode.Neutral;

  public Flywheel(FlywheelIO flywheelIO) {
    this.io = flywheelIO;

    // Configure dashboard
    SmartDashboard.putNumber("Flywheel/Voltage", 0.0);
    SmartDashboard.putData(
        "Flywheel/RunVoltage",
        Commands.runOnce(() -> runVolts(SmartDashboard.getNumber("Flywheel/Voltage", 0.0)), this));
  }

  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel", inputs);

    // Update alerts
    leaderDisconnectedAlert.set(!leaderConnectedDebouncer.calculate(inputs.leaderConnected));
    followerDisconnectedAlert.set(!followerConnectedDebouncer.calculate(inputs.followerConnected));
    leaderTempAlert.set(inputs.leaderTempCelsius > Constants.warningTempCelsius);
    followerTempAlert.set(inputs.followerTempCelsius > Constants.warningTempCelsius);

    // Update PID gains
    if (kP.hasChanged(hashCode()) | kD.hasChanged(hashCode()) | kS.hasChanged(hashCode())) {
      io.setPID(new SlotConfigs().withKP(kP.get()).withKD(kD.get()).withKS(kS.get()));
    }
  }

  /**
   * Run system at specified voltage.
   *
   * @param volts Voltage to run the motor at.
   */
  public void runVolts(double volts) {
    mode = ControlMode.Voltage;
    io.runVolts(volts);
    Logger.recordOutput("Shooter/Flywheel/SetpointVolts", volts, Volts);
  }

  /**
   * Run system to specified velocity.
   *
   * <p><b>Units:</b> Mechanism rotations per second.
   *
   * @param velocity Goal velocity.
   */
  public void runVelocity(double velocity) {
    setpoint = velocity;
    mode = ControlMode.Velocity;
    io.runVelocity(velocity, 0);
    Logger.recordOutput("Shooter/Flywheel/SetpointVelocity", setpoint, RotationsPerSecond);
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
        // TODO: this is bugged when setpoint is zero
        // TODO: decide if this should actually be based off percent.
      case Velocity ->
          (getVelocity() == 0.0)
              ? false
              : Math.abs((setpoint / getVelocity()) - 1) < setpointBandVelocity.get();
      default -> false;
    };
  }
}
