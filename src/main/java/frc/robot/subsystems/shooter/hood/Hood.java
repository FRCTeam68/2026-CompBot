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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ElasticUtil;
import frc.robot.util.ElasticUtil.Notification;
import frc.robot.util.ElasticUtil.NotificationLevel;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  // Positions
  @Getter private static final double maximum = 72;
  @Getter private static final double minimum = maximum - 26;
  @Getter private static final double underTrenchMinimum = maximum - 9;

  // PID gains
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Hood/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Hood/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Hood/kS");

  static {
    switch (Constants.getMode()) {
      case REAL, REPLAY -> {
        kP.initDefault(500);
        kD.initDefault(0.0);
        kS.initDefault(0.4);
      }
      case SIM -> {
        kP.initDefault(5);
        kD.initDefault(0.0);
        kS.initDefault(0.0);
      }
    }
  }

  // Motion magic gains
  private LoggedTunableNumber mmVelocity =
      new LoggedTunableNumber("Shooter/Hood/MotionMagic/Velocity", 100);
  private LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("Shooter/Hood/MotionMagic/Acceleration", 100);
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
  @Getter @Setter private boolean forceDown = false;

  public Hood(HoodIO hoodIO) {
    this.io = hoodIO;

    // Configure dashboard
    SmartDashboard.putData(
        "Shooter/ZeroHood",
        Commands.runOnce(() -> zero(), this).ignoringDisable(true).withName("ZeroHood"));
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
    if (prevInTrenchBox != (inTrenchBox.get() || forceDown)) {
      if (getElevation() < underTrenchMinimum || setpoint < underTrenchMinimum) {
        runElvation(setpoint);
      }
      prevInTrenchBox = (inTrenchBox.get() || forceDown);
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
        ((inTrenchBox.get() || forceDown))
            ? MathUtil.clamp(setpoint, underTrenchMinimum, maximum)
            : setpoint;
    mode = ControlMode.Position;

    io.runPosition(setpointAdjusted - maximum, 0);

    Logger.recordOutput("Shooter/Hood/SetpointPosition", setpoint, Degrees);
    Logger.recordOutput("Shooter/Hood/SetpointPositionAdjusted", setpointAdjusted, Degrees);
  }

  /** Stop motor with neutral output. */
  public void stop() {
    mode = ControlMode.Neutral;
    io.stop();
  }

  /** Set the current mechanism position to zero. */
  public void zero() {
    io.setPosition(0);
    ElasticUtil.sendNotification(
        new Notification(
            NotificationLevel.INFO,
            "Hood Zeroed",
            "Hood position should be at the maximum elevation."));
  }

  /**
   * Elevation of the system in mechanism degrees.
   *
   * @return Elevation.
   */
  @AutoLogOutput(key = "Shooter/Hood/ElevationMeasured", unit = "Degrees")
  public double getElevation() {
    return maximum + inputs.positionDeg;
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
    if (mode == ControlMode.Position) {
      return Math.abs(setpoint - getElevation()) < setpointBandPosition.get();
    }
    return false;
  }
}
