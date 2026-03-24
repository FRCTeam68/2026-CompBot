package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import frc.robot.util.VirtualPD;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  // PID gains
  private final LoggedTunableNumber[] kP =
      new LoggedTunableNumber[] {
        new LoggedTunableNumber("Shooter/Flywheel/Slot0-Velocity/kP", 3.0),
        new LoggedTunableNumber("Shooter/Flywheel/Slot1-BangBang/kP", 10)
      };
  private final LoggedTunableNumber[] kD =
      new LoggedTunableNumber[] {
        new LoggedTunableNumber("Shooter/Flywheel/Slot0-Velocity/kD", 0.0),
        new LoggedTunableNumber("Shooter/Flywheel/Slot1-BangBang/kD", 0.0)
      };
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Flywheel/kS", 2.5);

  // Setpoint tunable numbers
  private final LoggedTunableNumber setpointBandVelocity =
      new LoggedTunableNumber("Shooter/Flywheel/VelocitySetpointBand", 2.5);
  private final LoggedTunableNumber setpointDebouncerTime =
      new LoggedTunableNumber("Shooter/Flywheel/SetpointDebouncerTime", 0.5);

  // Bang-Bang tunable numbers
  private final LoggedTunableNumber bangBangVolts =
      new LoggedTunableNumber("Shooter/Flywheel/BangBangVolts", 12);
  private final LoggedTunableNumber bangBangTolerance =
      new LoggedTunableNumber("Shooter/Flywheel/BangBangTolerance", 2);

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
  private final Debouncer atSetpointDebouncer = new Debouncer(0.0, DebounceType.kFalling);

  private final FlywheelIO io;
  protected final FlyWheelIOInputsAutoLogged inputs = new FlyWheelIOInputsAutoLogged();
  @Getter private double setpoint = 0.0;
  @Getter private ControlMode mode = ControlMode.Neutral;

  @AutoLogOutput(key = "Shooter/Flywheel/BumpVelocity", unit = "RotsPerSec")
  public double bumpVelocity = 0.0;

  public Flywheel(FlywheelIO flywheelIO) {
    this.io = flywheelIO;

    VirtualPD.registerMotor(
        () -> Watts.of(Math.abs(inputs.leaderSupplyCurrentAmps * inputs.leaderAppliedVoltage)),
        "Flywheel");
    VirtualPD.registerMotor(
        () -> Watts.of(Math.abs(inputs.followerSupplyCurrentAmps * inputs.followerAppliedVoltage)),
        "Flywheel");
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

    Logger.recordOutput("Shooter/Flywheel/ControlMode", mode.toString());

    if (mode == ControlMode.BangBang) bangBangControl();

    // Update PID gains
    if (kP[0].hasChanged(hashCode())
        | kP[1].hasChanged(hashCode())
        | kD[0].hasChanged(hashCode())
        | kD[1].hasChanged(hashCode())
        | kS.hasChanged(hashCode())) {
      io.setPID(
          new SlotConfigs().withKP(kP[0].get()).withKD(kD[0].get()).withKS(kS.get()),
          new SlotConfigs().withKP(kP[1].get()).withKD(kD[1].get()).withKS(kS.get()));
    }

    // Update atSetpoint debouncer
    if (setpointDebouncerTime.hasChanged(hashCode())) {
      atSetpointDebouncer.setDebounceTime(setpointDebouncerTime.get());
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
   * Run system to specified velocity with simple PID control.
   *
   * <p><b>Units:</b> Mechanism rotations per second.
   *
   * @param velocity Goal velocity.
   */
  public void runVelocity(double velocity) {
    setpoint = velocity + bumpVelocity;
    mode = ControlMode.Velocity;
    io.runVelocity(velocity, 0);
    Logger.recordOutput("Shooter/Flywheel/SetpointVelocity", setpoint, RotationsPerSecond);
  }

  /**
   * Run system to specified velocity with a Bang-Bang Controller.
   *
   * <p><b>Units:</b> Mechanism rotations per second.
   *
   * @param velocity Goal velocity.
   */
  public void runBangBang(double velocity) {
    setpoint = velocity + bumpVelocity;
    mode = ControlMode.BangBang;
    Logger.recordOutput("Shooter/Flywheel/BangBangSetpoint", setpoint, RotationsPerSecond);
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
      case Velocity, BangBang ->
          atSetpointDebouncer.calculate(
              Math.abs(setpoint - getVelocity()) < setpointBandVelocity.get());
      default -> false;
    };
  }

  /**
   * Controls the Bang-Bang controller. This should be called periodically if using the Bang-Bang
   * controller.
   */
  private void bangBangControl() {
    final boolean notNearSetpoint = getVelocity() < setpoint - bangBangTolerance.get();

    if (notNearSetpoint) {
      io.runVolts(bangBangVolts.get());
    } else {
      io.runVelocity(setpoint, 1);
    }

    Logger.recordOutput(
        "Shooter/Flywheel/SetpointVolts", notNearSetpoint ? bangBangVolts.get() : 0, Volts);
    Logger.recordOutput(
        "Shooter/Flywheel/SetpointVelocity", notNearSetpoint ? 0 : setpoint, RotationsPerSecond);
    Logger.recordOutput(
        "Shooter/Flywheel/BangBangControlMode", notNearSetpoint ? "Voltage" : "Velocity");
  }
}
