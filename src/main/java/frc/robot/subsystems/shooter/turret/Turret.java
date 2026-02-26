package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  // Positions
  @Getter private static final double minimum = 0;
  @Getter private static final double maximum = 360;

  // PID gains
  private LoggedTunableNumber kP0 = new LoggedTunableNumber("Shooter/Turret/Slot0/kP", 70);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber("Shooter/Turret/Slot0/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber("Shooter/Turret/Slot0/kS", 0.3);

  // Motion magic gains
  private LoggedTunableNumber mmVelocity =
      new LoggedTunableNumber("Shooter/Turret/MotionMagic/Velocity", 400);
  private LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("Shooter/Turret/MotionMagic/Acceleration", 9999);
  private LoggedTunableNumber mmJerk =
      new LoggedTunableNumber("Shooter/Turret/MotionMagic/Jerk", 0);

  // Error bands
  private LoggedTunableNumber setpointBandPosition =
      new LoggedTunableNumber("Shooter/Turret/PositionSetpointBand", 10);
  private static final double ambiguousBand = 20;

  // Alerts
  private final Alert motorDisconnectedAlert =
      new Alert("Turret motor disconnected!", AlertType.kError);
  private final Alert turretCancoderDisconnectedAlert =
      new Alert("Turret cancoder disconnected!", AlertType.kError);
  private final Alert motorTempAlert = new Alert("Turret motor is too hot.", AlertType.kWarning);
  private final Alert posistionAmbiguousAlert =
      new Alert(
          "Turret position ambiguous! Turret will not move until position is verified.",
          AlertType.kError);

  // Debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Debouncer cancoderConnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private final TurretIO io;
  protected final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  @Getter private double setpoint = 0.0;
  @Getter private ControlMode mode = ControlMode.Neutral;
  private static boolean posistionAmbiguous = false;

  public Turret(TurretIO io) {
    this.io = io;

    // Check if turret position could be ambiguous
    if (Constants.getMode() == Mode.REAL) {
      this.io.updateInputs(inputs);
      posistionAmbiguous =
          getPosition() < (ambiguousBand / 2) || getPosition() > 360 - (ambiguousBand / 2);
    }

    // Configure dashboard
    SmartDashboard.putData(
        "Shooter/DisambiguateTurret",
        Commands.runOnce(() -> disambiguate())
            .ignoringDisable(true)
            .withName("DisambiguateTurret"));
  }

  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret", inputs);

    // Update alerts
    motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    turretCancoderDisconnectedAlert.set(
        !cancoderConnectedDebouncer.calculate(inputs.cancoderConnected));
    motorTempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);
    posistionAmbiguousAlert.set(posistionAmbiguous);

    // Log setpoint
    Logger.recordOutput(
        "Shooter/Turret/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput(
        "Shooter/Turret/SetpointPositionDeg", (mode == ControlMode.Position) ? setpoint : 0);

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
    if (posistionAmbiguous == false) {
      setpoint = volts;
      mode = ControlMode.Voltage;
      io.runVolts(volts);
    }
  }

  /**
   * Run system to specified position.
   *
   * <p><b>Units:</b> Mechanism degrees.
   *
   * @param degrees Goal position.
   * @param slot PID gain slot to use during motion.
   */
  public void runPosition(double degrees, int slot) {
    if (posistionAmbiguous == false) {
      setpoint = MathUtil.inputModulus(degrees, minimum, maximum);
      mode = ControlMode.Position;
      io.runPosition(setpoint, slot);
    }
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

  public void disambiguate() {
    if (getAbsolutePosition() > (ambiguousBand / 2)
        && getAbsolutePosition() < 360 - (ambiguousBand / 2)
        && inputs.velocityDegPerSec < 0.1) {
      io.setPosition(getAbsolutePosition());
      posistionAmbiguous = false;
    }
  }

  /**
   * Position of the system in mechanism degrees.
   *
   * @return Position.
   */
  public double getPosition() {
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
  @AutoLogOutput(key = "Shooter/Turret/atSetpoint")
  public boolean atSetpoint() {
    return switch (mode) {
      case Position -> Math.abs(setpoint - getPosition()) < setpointBandPosition.get();
      default -> false;
    };
  }
}
