package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.Color;
import frc.robot.subsystems.lights.Lights.LEDSegment;
import frc.robot.util.ElasticUtil;
import frc.robot.util.ElasticUtil.Notification;
import frc.robot.util.ElasticUtil.NotificationLevel;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import frc.robot.util.VirtualPD;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  // Positions
  @Getter private static final double minimum = 30;
  @Getter private static final double maximum = 360;

  // PID gains
  private LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Turret/kP", 70);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Turret/kD", 0);
  private LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Turret/kS", .3);

  // Motion magic gains
  private LoggedTunableNumber mmVelocity =
      new LoggedTunableNumber("Shooter/Turret/MotionMagic/Velocity", 800);
  private LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("Shooter/Turret/MotionMagic/Acceleration", 5000);
  private LoggedTunableNumber mmJerk =
      new LoggedTunableNumber("Shooter/Turret/MotionMagic/Jerk", 0);

  // Error bands
  private LoggedTunableNumber setpointBandPosition =
      new LoggedTunableNumber("Shooter/Turret/PositionSetpointBand", 10);
  private static final double ambiguousBand = 30;

  // Alerts
  private final Alert motorDisconnectedAlert =
      new Alert("Turret motor disconnected!", AlertType.kError);
  private final Alert turretCancoderDisconnectedAlert =
      new Alert("Turret cancoder disconnected!", AlertType.kError);
  private final Alert motorTempAlert = new Alert("Turret motor is too hot.", AlertType.kWarning);
  private final Alert posistionAmbiguousAlert =
      new Alert(
          "Turret position ambiguous! Turret will not move until position is Disambiguated.",
          AlertType.kError);

  // Debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Debouncer cancoderConnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private final TurretIO io;
  protected final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Lights lights;
  private final LEDSegment ledSegment = new LEDSegment(1, 1, 0);
  @Getter private double setpoint = 0.0;
  @Getter private ControlMode mode = ControlMode.Neutral;
  private static boolean posistionAmbiguous = false;

  @AutoLogOutput(key = "Shooter/Turret/BumpAngle", unit = "Degrees")
  public double bumpAngle = 0.0;

  public Turret(Lights lights, TurretIO io) {
    this.lights = lights;
    this.io = io;

    VirtualPD.registerMotor(
        () -> Watts.of(Math.abs(inputs.supplyCurrentAmps * inputs.appliedVoltage)), "Turret");

    // Check if turret position could be ambiguous
    if (Constants.getMode() != Mode.SIM) {
      periodic();
      posistionAmbiguous =
          getPosition() < (ambiguousBand / 2) || getPosition() > 360 - (ambiguousBand / 2);
    }

    // Configure dashboard
    SmartDashboard.putData(
        "Shooter/DisambiguateTurret",
        Commands.runOnce(() -> disambiguate())
            .ignoringDisable(true)
            .withName("DisambiguateTurret"));
    SmartDashboard.putData(
        "Shooter/ZeroTurret",
        Commands.runOnce(() -> zero(), this).ignoringDisable(true).withName("ZeroTurret"));
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

    if (DriverStation.isDisabled() || Constants.tuningMode) {
      if (posistionAmbiguous) {
        lights.setSolidColor(Color.Dim.RED, ledSegment);
      } else {
        lights.setSolidColor(Color.Dim.GREEN, ledSegment);
      }
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
    setpoint = volts;
    mode = ControlMode.Voltage;
    if (posistionAmbiguous == false) {
      io.runVolts(volts);
    }
    Logger.recordOutput("Shooter/Turret/SetpointVolts", volts, Volts);
  }

  /**
   * Run system to specified position.
   *
   * <p><b>Units:</b> Mechanism degrees.
   *
   * @param degrees Goal position.
   */
  public void runPosition(double degrees) {
    setpoint = MathUtil.inputModulus(degrees + bumpAngle, 0.0, 360.0);
    mode = ControlMode.Position;
    if (posistionAmbiguous == false) {
      io.runPosition(setpoint, 0);
    }
    Logger.recordOutput("Shooter/Turret/SetpointPosition", setpoint, Degrees);
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
            "Turret Zeroed",
            "Turret should be facing forward with the cable completely unwound."));
  }

  public void disambiguate() {
    if (getAbsolutePosition() > (ambiguousBand / 2)
        && getAbsolutePosition() < 360 - (ambiguousBand / 2)
        && inputs.velocityDegPerSec < 0.01) {
      io.setPosition(getAbsolutePosition());
      posistionAmbiguous = false;
      ElasticUtil.sendNotification(
          new Notification(
              NotificationLevel.INFO,
              "Turret Position Disambiguated",
              "Turret will now run normally."));
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
