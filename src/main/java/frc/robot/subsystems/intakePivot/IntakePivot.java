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
  @Getter private static final double packaged = 0;
  @Getter private static final double extended = 0.23;

  private final IntakePivotIO io;
  protected final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  // TODO: Rename the debouncers to have more descriptive names
  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Debouncer disconnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);

  // TODO: Remane the motor disconnected alert to have a more descriptive name
  private final Alert disconnectedAlert =
      new Alert("Intake pivot motor disconnected!", AlertType.kError);
  private final Alert cancoderDisconnectedAlert =
      new Alert("Intake pivot cancoder disconnected!", AlertType.kError);
  private final Alert tempAlert = new Alert("Intake pivot motor is too hot.", AlertType.kWarning);

  private LoggedTunableNumber kP0 = new LoggedTunableNumber("IntakePivot/Slot0/kP", 10);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber("IntakePivot/Slot0/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber("IntakePivot/Slot0/kS", 0);

  private LoggedTunableNumber mmVelocity = new LoggedTunableNumber("IntakePivot/Velocity", 0);
  private LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("IntakePivot/Acceleration", 0);
  private LoggedTunableNumber mmJerk = new LoggedTunableNumber("IntakePivot/Jerk", 0);

  private LoggedTunableNumber setpointBandPosition =
      new LoggedTunableNumber("IntakePivot/SetpointBand", 0);

  @Getter private double setpoint = 0.0;

  @Getter private ControlMode mode = ControlMode.Neutral;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;

    // Dashboard controls
    // TODO: Add names to these commands. Start the name with Dashboard so we know thats what these
    // are.
    SmartDashboard.putData(
        "IntakePivot/Extend", Commands.runOnce(() -> runPosition(extended, 0), this));
    SmartDashboard.putData(
        "IntakePivot/Retract", Commands.runOnce(() -> runPosition(packaged, 0), this));
    SmartDashboard.putData("IntakePivot/Zero", Commands.runOnce(() -> zero(), this));
  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);
    disconnectedAlert.set(!connectedDebouncer.calculate(inputs.motorConnected));
    cancoderDisconnectedAlert.set(!disconnectedDebouncer.calculate(inputs.cancoderConnected));
    tempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);

    // Log setpoint
    Logger.recordOutput("IntakePivot/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput(
        "IntakePivot/SetpointPositionRots", (mode == ControlMode.Position) ? setpoint : 0);

    // Update PID
    if (kP0.hasChanged(hashCode()) | kD0.hasChanged(hashCode()) | kS0.hasChanged(hashCode())) {
      io.setPID(new SlotConfigs().withKP(kP0.get()).withKD(kD0.get()).withKS(kS0.get()));
    }

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
   * @param position Goal position.
   * @param slot PID gain slot to use during motion.
   */
  public void runPosition(double position, int slot) {
    setpoint = position;
    mode = ControlMode.Position;
    io.runPosition(position, slot);
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
      case Position -> Math.abs(setpoint - inputs.positionRots) < setpointBandPosition.get();
      default -> false;
    };
  }
}
