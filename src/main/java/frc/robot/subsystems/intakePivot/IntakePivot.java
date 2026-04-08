package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import frc.robot.util.VirtualPD;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  // Positions
  @Getter private static final double extended = 0;
  @Getter private static final double agitate = -0.161;

  @Getter private static final double packaged = -0.284;
  // -0.278 is target but it is coming up short at -0.271, so add 0.07

  @Getter private static final double intakeForwardExtension = Units.inchesToMeters(12);

  // PID gains
  private final LoggedTunableNumber[] kP =
      new LoggedTunableNumber[] {
        new LoggedTunableNumber("IntakePivot/Slot0-Deploy/kP", 100),
        new LoggedTunableNumber("IntakePivot/Slot1-Retract/kP", 800),
        new LoggedTunableNumber("IntakePivot/Slot2-DeployFirst/kP", 650)
      };
  private final LoggedTunableNumber[] kD =
      new LoggedTunableNumber[] {
        new LoggedTunableNumber("IntakePivot/Slot0-Deploy/kD", 0),
        new LoggedTunableNumber("IntakePivot/Slot1-Retract/kD", 0),
        new LoggedTunableNumber("IntakePivot/Slot2-DeployFirst/kD", 0)
      };
  private final LoggedTunableNumber kS = new LoggedTunableNumber("IntakePivot/kS", 60);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("IntakePivot/kG", -80);

  // Setpoint band
  private final LoggedTunableNumber setpointBandPosition =
      new LoggedTunableNumber("IntakePivot/SetpointBand", 0.1);

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

    VirtualPD.registerMotor(
        () -> Watts.of(Math.abs(inputs.supplyCurrentAmps * inputs.appliedVoltage)), "IntakePivot");

    // Configure dashboard
    SmartDashboard.putData(
        "IntakePivot/Zero", Commands.runOnce(() -> zero(), this).withName("DashboardIntakeZero"));
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

    // Update PID gains
    if (kP[0].hasChanged(hashCode())
        | kD[0].hasChanged(hashCode())
        | kP[1].hasChanged(hashCode())
        | kD[1].hasChanged(hashCode())
        | kP[2].hasChanged(hashCode())
        | kD[2].hasChanged(hashCode())
        | kS.hasChanged(hashCode())
        | kG.hasChanged(hashCode())) {
      io.setPID(
          new SlotConfigs().withKP(kP[0].get()).withKD(kD[0].get()).withKS(kS.get()),
          new SlotConfigs()
              .withKP(kP[1].get())
              .withKD(kD[1].get())
              .withKS(kS.get())
              .withKG(kG.get()),
          new SlotConfigs().withKP(kP[2].get()).withKD(kD[2].get()).withKS(kS.get()));
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
    Logger.recordOutput("IntakePivot/SetpointVolts", volts, Volts);
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
    Logger.recordOutput("IntakePivot/SetpointPosition", setpoint, Rotations);
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

  @AutoLogOutput(key = "IntakePivot/isRetracted")
  public boolean isRetracted() {
    return getPosition() < (packaged + 0.08);
  }

  @AutoLogOutput(key = "IntakePivot/isExtended")
  public boolean isExtended() {
    return getPosition() > (extended - 0.08);
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
    if (mode == ControlMode.Position) {
      return Math.abs(setpoint - getPosition()) < setpointBandPosition.get();
    }
    return false;
  }

  /** Configure dashboard tuning controls for manual control. */
  public void configureDashboardControls() {
    SmartDashboard.putData(
        "IntakePivot/Extend",
        Commands.runOnce(() -> runPosition(extended, 0), this)
            .withName("DashboardIntakePivotExtend"));
    SmartDashboard.putData(
        "IntakePivot/Retract",
        Commands.runOnce(() -> runPosition(packaged, 1), this)
            .withName("DashboardIntakePivotRetract"));
  }
}
