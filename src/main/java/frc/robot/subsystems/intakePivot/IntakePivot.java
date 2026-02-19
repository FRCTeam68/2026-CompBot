package frc.robot.subsystems.intakePivot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
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

public class IntakePivot extends SubsystemBase {
  // Positions
  @Getter private static final double packaged = 0;
  @Getter private static final double extended = 0.23;

  // PID gains
  private final LoggedTunableNumber kP0 = new LoggedTunableNumber("IntakePivot/Slot0/kP", 10);
  private final LoggedTunableNumber kD0 = new LoggedTunableNumber("IntakePivot/Slot0/kD", 0);
  private final LoggedTunableNumber kS0 = new LoggedTunableNumber("IntakePivot/Slot0/kS", 0);

  // Motion magic gains
  private final LoggedTunableNumber mmVelocity = new LoggedTunableNumber("IntakePivot/Velocity", 0);
  private final LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("IntakePivot/Acceleration", 0);
  private final LoggedTunableNumber mmJerk = new LoggedTunableNumber("IntakePivot/Jerk", 0);

  // setpoint band
  private final LoggedTunableNumber setpointBandPosition =
      new LoggedTunableNumber("IntakePivot/SetpointBand", 0);

  // Alerts
  private final Alert motorDisconnectedAlert =
      new Alert("Intake pivot motor disconnected!", AlertType.kError);
  private final Alert cancoderDisconnectedAlert =
      new Alert("Intake pivot cancoder disconnected!", AlertType.kError);
  private final Alert motorTempAlert =
      new Alert("Intake pivot motor is too hot.", AlertType.kWarning);

  // Debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Debouncer cancoderDisconnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private final IntakePivotIO io;
  protected final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
  @Getter private double setpoint = 0.0;
  @Getter private ControlMode mode = ControlMode.Neutral;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;

    // Configure dashboard
    SmartDashboard.putData(
        "IntakePivot/Extend",
        Commands.runOnce(() -> runPosition(extended, 0)).withName("DashboardIntakePivotExtend"));
    SmartDashboard.putData(
        "IntakePivot/Retract",
        Commands.runOnce(() -> runPosition(packaged, 0)).withName("DashboardIntakePivotRetract"));
    SmartDashboard.putData(
        "IntakePivot/Zero", Commands.runOnce(() -> zero()).withName("DashboardIntakePivotZero"));
  }

  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    // Update alerts
    motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    cancoderDisconnectedAlert.set(
        !cancoderDisconnectedDebouncer.calculate(inputs.cancoderConnected));
    motorTempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);

    // Log setpoint
    Logger.recordOutput("IntakePivot/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput(
        "IntakePivot/SetpointPositionRots", (mode == ControlMode.Position) ? setpoint : 0);

    // Update PID gains
    if (kP0.hasChanged(hashCode()) | kD0.hasChanged(hashCode()) | kS0.hasChanged(hashCode())) {
      io.setPID(new SlotConfigs().withKP(kP0.get()).withKD(kD0.get()).withKS(kS0.get()));
    }

    // Update motion magic gains
    if (mmVelocity.hasChanged(hashCode())
        | mmAcceleration.hasChanged(hashCode())
        | mmJerk.hasChanged(hashCode())) {
      io.setMotionMagic(
          new MotionMagicConfigs()
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
   * <p><b>Units:</b> Mechanism rotations.
   *
   * @param rotations Goal position.
   * @param slot PID gain slot to use during motion.
   */
  public void runPosition(double rotations, int slot) {
    setpoint = rotations;
    mode = ControlMode.Position;
    io.runPosition(rotations, slot);
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

  /**
   * Velocity of the system in mechanism rotations per second.
   *
   * @return Velocity.
   */
  public double getVelocity() {
    return inputs.velocityRotsPerSec;
  }

  /**
   * Position of the system in mechanism rotations.
   *
   * @return Position.
   */
  public double getPosition() {
    return inputs.positionRots;
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
  @AutoLogOutput(key = "IntakePivot/atSetpoint")
  public boolean atSetpoint() {
    return switch (mode) {
      case Position -> Math.abs(setpoint - getPosition()) < setpointBandPosition.get();
      default -> false;
    };
  }
}
