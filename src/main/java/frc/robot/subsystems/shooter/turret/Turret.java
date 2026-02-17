package frc.robot.subsystems.shooter.turret;

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

public class Turret extends SubsystemBase {
  private final TurretIO io;
  protected final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Alert turretMotorDisconnectedAlert =
      new Alert("Turret motor disconnected!", AlertType.kError);
  private final Alert turretMotorTempAlert =
      new Alert("Turret motor is too hot.", AlertType.kWarning);
  private final Debouncer turretMotorDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private LoggedTunableNumber kP0 = new LoggedTunableNumber("Turret/Slot0/kP", 20);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber("Turret/Slot0/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber("Turret/Slot0/kS", 0);

  private LoggedTunableNumber mmVelocity =
      new LoggedTunableNumber("Turret/MotionMagic/Velocity", 0);
  private LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("Turret/MotionMagic/Acceleration", 0);
  private LoggedTunableNumber mmJerk = new LoggedTunableNumber("Turret/MotionMagic/Jerk", 0);

  private LoggedTunableNumber setpointBandPosition =
      new LoggedTunableNumber("Turret/PositionSetpointBand", 0);

  @Getter private double setpoint = 0.0;

  @Getter private ControlMode mode = ControlMode.Neutral;

  public Turret(TurretIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    turretMotorDisconnectedAlert.set(!turretMotorDebouncer.calculate(inputs.connected));
    turretMotorTempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);

    Logger.recordOutput("Turret/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput(
        "Turret/SetpointPositionRots", (mode == ControlMode.Position) ? setpoint : 0);

    // Update tunable numbers
    if (kP0.hasChanged(hashCode()) || kD0.hasChanged(hashCode()) || kS0.hasChanged(hashCode())) {

      io.setPID(new SlotConfigs().withKP(kP0.get()).withKD(kD0.get()).withKS(kS0.get()));
    }

    if (mmVelocity.hasChanged(hashCode())
        || mmAcceleration.hasChanged(hashCode())
        || mmJerk.hasChanged(hashCode())) {
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

  /**
   * Set goal position in mechanism rotations
   *
   * @param position Goal position
   */
  public void runPosition(double position, int slot) {
    mode = ControlMode.Position;
    io.runPosition(position, 0);
  }

  /** Stop motor */
  public void stop() {
    mode = ControlMode.Neutral;
    io.stop();
  }

  /** Set the current mechanism position to zero */
  public void zero() {
    io.setPosition(0);
  }

  /**
   * Set the current mechanism position
   *
   * @param rotations Position in mechanism rotations
   */
  public void setPosition(double rotations) {
    io.setPosition(rotations);
  }

  /**
   * Position of the mechanism in degrees of elevation
   *
   * @return Elevation of the wrist
   */
  public double getPosition() {
    return inputs.positionRots;
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
    return inputs.torqueCurrentAmps;
  }

  /**
   * Check if mechanism is at goal position with error of setpointBandPosition
   *
   * @return True if in position control mode and mechanism is at goal position, false otherwise
   */
  @AutoLogOutput(key = "Hood/atSetpoint")
  public boolean atSetpoint() {
    return switch (mode) {
      case Position -> Math.abs(setpoint - inputs.positionRots) < setpointBandPosition.get();
      default -> false;
    };
  }
}
