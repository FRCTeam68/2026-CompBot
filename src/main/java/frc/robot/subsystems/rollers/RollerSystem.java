package frc.robot.subsystems.rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class RollerSystem extends SubsystemBase {
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();

  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Alert disconnectedAlert;
  private final Alert tempAlert;
  private String loggerKey = "";

  @Getter private double setpointVolts = 0.0;

  /**
   * Creates a generic roller system. Rollers can only be controlled through voltage.
   *
   * <p>When using the name for logging, spaces are removed and each word is capitalized.
   *
   * <p>Examples:
   *
   * <blockquote>
   *
   * <pre>
   * name: "Feeder upper"
   * alert: "Feeder upper motor disconnected!"
   * logger: "FeederUpper"
   * </pre>
   *
   * </blockquote>
   *
   * @param name Name of the system used for alerts and logging.
   * @param io IO implementation for the system.
   */
  public RollerSystem(String name, RollerSystemIO io) {
    this.io = io;

    disconnectedAlert = new Alert(name + " motor disconnected!", AlertType.kError);
    tempAlert = new Alert(name + " motor is too hot.", AlertType.kWarning);
    String[] nameSplits = name.split(" ");
    for (String nameSplit : nameSplits) {
      loggerKey =
          loggerKey.concat(nameSplit.substring(0, 1).toUpperCase().concat(nameSplit.substring(1)));
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(loggerKey, inputs);
    disconnectedAlert.set(!connectedDebouncer.calculate(inputs.connected));
    tempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);
  }

  /** Run roller at volts */
  public void runVolts(double volts) {
    setpointVolts = volts;
    io.runVolts(setpointVolts);
    Logger.recordOutput(loggerKey + "/SetpointVolts", setpointVolts);
  }

  /** Stop roller */
  public void stop() {
    setpointVolts = 0.0;
    io.stop();
    Logger.recordOutput(loggerKey + "/SetpointVolts", 0.0);
  }

  /**
   * @return Velocity of roller in mechanism rotations per second
   */
  public double getVelocity() {
    return inputs.velocityRotsPerSec;
  }

  /**
   * @return Torque current of roller
   */
  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }
}
