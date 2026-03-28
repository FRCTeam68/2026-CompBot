package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.VirtualPD;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class RollerSystem extends SubsystemBase {
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();

  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Alert disconnectedAlert;
  private final Alert tempAlert;
  private final String loggerKey;

  @Getter private double setpointVolts = 0.0;

  /**
   * Creates a generic roller system. Rollers can only be controlled through voltage.
   *
   * <p>When using the name for logging key, spaces are removed and each word is capitalized.
   *
   * <p>Examples:
   *
   * <blockquote>
   *
   * <pre>
   * name: "Feeder upper"
   * alert text: "Feeder upper motor disconnected!"
   * logger key: "FeederUpper"
   * </pre>
   *
   * </blockquote>
   *
   * @param name Name of the system used for alerts and logging.
   * @param io IO implementation for the system.
   */
  public RollerSystem(String name, RollerSystemIO io) {
    this.io = io;

    VirtualPD.registerMotor(
        () -> Watts.of(Math.abs(inputs.supplyCurrentAmps * inputs.appliedVoltage)), name);

    // Create alert text
    disconnectedAlert = new Alert(name + " motor disconnected!", AlertType.kError);
    tempAlert = new Alert(name + " motor is too hot.", AlertType.kWarning);

    // Create logger key
    final String[] nameSplits = name.split(" ");
    String tempKey = "";
    for (String nameSplit : nameSplits) {
      tempKey =
          tempKey.concat(nameSplit.substring(0, 1).toUpperCase().concat(nameSplit.substring(1)));
    }
    loggerKey = tempKey;
  }

  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs(loggerKey, inputs);

    // Update alerts
    disconnectedAlert.set(!connectedDebouncer.calculate(inputs.connected));
    tempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);
  }

  /**
   * Run system at specified voltage.
   *
   * @param volts Voltage to run the motor at.
   */
  public void runVolts(double volts) {
    setpointVolts = volts;
    io.runVolts(setpointVolts);
    Logger.recordOutput(loggerKey + "/SetpointVolts", setpointVolts, Volts);
  }

  /** Stop motor with neutral output. */
  public void stop() {
    setpointVolts = 0.0;
    io.stop();
    Logger.recordOutput(loggerKey + "/SetpointVolts", 0.0, Volts);
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
   * @return Motor torque current.
   */
  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  /** Configure dashboard tuning controls for manual control. */
  public void configureDashboardControls() {
    SmartDashboard.putNumber(loggerKey + "/Voltage", 0.0);
    SmartDashboard.putData(
        loggerKey + "/RunVoltage",
        Commands.runOnce(
                () -> runVolts(SmartDashboard.getNumber(loggerKey + "/Voltage", 0.0)), this)
            .withName("Dashboard" + loggerKey + "RunVolts"));
  }
}
