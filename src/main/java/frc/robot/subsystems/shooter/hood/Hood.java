package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  // Positions
  @Getter private static final double maximum = 72;
  @Getter private static final double minimum = maximum - 26;
  @Getter private static final double underTrenchMinimum = maximum - 9;

  // PID gains
  private LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Hood/kP", 20);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Hood/kD", 0);
  private LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Hood/kS", 0);

  // Motion magic gains
  private LoggedTunableNumber mmVelocity =
      new LoggedTunableNumber("Shooter/Hood/MotionMagic/Velocity", 0);
  private LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("Shooter/Hood/MotionMagic/Acceleration", 0);
  private LoggedTunableNumber mmJerk = new LoggedTunableNumber("Shooter/Hood/MotionMagic/Jerk", 0);

  // Setpoint band
  private LoggedTunableNumber setpointBandPosition =
      new LoggedTunableNumber("Shooter/Hood/PositionSetpointBand", 2);

  // Alerts
  private final Alert motorDisconnectedAlert =
      new Alert("Hood motor disconnected!", AlertType.kError);
  private final Alert hoodCancoderDisconnectedAlert =
      new Alert("Hood CANcoder disconnected!", AlertType.kError);
  private final Alert motorTempAlert = new Alert("Hood motor is too hot.", AlertType.kWarning);

  // Debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Debouncer cancoderConnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private final HoodIO io;
  protected final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private Supplier<Boolean> inTrenchBox = () -> false;
  @Getter private double setpoint = 0.0;
  private double setpointAdjusted = 0.0;
  @Getter private ControlMode mode = ControlMode.Neutral;
  private boolean prevInTrenchBox = false;

  public Hood(HoodIO hoodIO) {
    this.io = hoodIO;

    // Set current position
    periodic();
    io.setPosition(maximum + getAbsolutePosition());
  }

  public void initInTrenchBoxSupplier(Supplier<Boolean> inTrenchBox) {
    this.inTrenchBox = inTrenchBox;
  }

  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Hood", inputs);

    // Update alerts
    motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    hoodCancoderDisconnectedAlert.set(
        !cancoderConnectedDebouncer.calculate(inputs.cancoderConnected));
    motorTempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);

    // Run hood if entering/leaving trench box
    if (prevInTrenchBox != inTrenchBox.get()) {
      if (getElevation() < underTrenchMinimum || setpoint < underTrenchMinimum) {
        runElvation(setpoint);
      }
      prevInTrenchBox = inTrenchBox.get();
    }

    // Update PID gains
    if (kP.hasChanged(hashCode()) | kD.hasChanged(hashCode()) | kS.hasChanged(hashCode())) {
      io.setPID(new SlotConfigs().withKP(kP.get()).withKD(kD.get()).withKS(kS.get()));
    }

    // Update motion magic gains
    if (mmVelocity.hasChanged(hashCode())
        | mmAcceleration.hasChanged(hashCode())
        | mmJerk.hasChanged(hashCode())) {
      io.setMotionMagic(
          new MotionMagicConfigs()
              .withMotionMagicCruiseVelocity(Units.degreesToRotations(mmVelocity.get()))
              .withMotionMagicAcceleration(Units.degreesToRotations(mmAcceleration.get()))
              .withMotionMagicJerk(Units.degreesToRotations(mmJerk.get())));
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
    Logger.recordOutput("Shooter/Hood/SetpointVolts", volts, Volts);
  }

  /**
   * Run system to specified elevation.
   *
   * <p><b>Units:</b> Mechanism degrees.
   *
   * @param elevation Goal elevation.
   */
  public void runElvation(double elevation) {
    setpoint = MathUtil.clamp(elevation, minimum, maximum);
    setpointAdjusted =
        (inTrenchBox.get()) ? MathUtil.clamp(setpoint, underTrenchMinimum, maximum) : setpoint;
    mode = ControlMode.Position;

    io.runPosition(setpointAdjusted, 0);

    Logger.recordOutput("Shooter/Hood/SetpointPosition", setpoint, Degrees);
    Logger.recordOutput("Shooter/Hood/SetpointPositionAdjusted", setpointAdjusted, Degrees);
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
    return inputs.positionDeg;
  }

  /**
   * Absolute position of the system in mechanism degrees.
   *
   * @return Absolute position.
   */
  public double getAbsolutePosition() {
    return inputs.absolutePositionDeg;
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
}
