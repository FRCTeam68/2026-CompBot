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
  private final String name;
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();

  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Alert disconnectedAlert;
  private final Alert tempAlert;

  @Getter private double setpointVolts = 0.0;

  public RollerSystem(String name, RollerSystemIO io) {
    this.name = name;
    this.io = io;

    disconnectedAlert = new Alert(name + " motor disconnected!", AlertType.kError);
    tempAlert = new Alert(name + " motor is too hot.", AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnectedAlert.set(!connectedDebouncer.calculate(inputs.connected));
    tempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);
  }

  /** Run roller at volts */
  public void runVolts(double volts) {
    setpointVolts = volts;
    io.runVolts(setpointVolts);
    Logger.recordOutput(name + "/SetpointVolts", setpointVolts);
  }

  /** Stop roller */
  public void stop() {
    setpointVolts = 0.0;
    io.stop();
    Logger.recordOutput(name + "/SetpointVolts", 0.0);
  }

  /**
   * @return Velocity of roller in mechanism rotations per second
   */
  public double getVelocity() {
    return inputs.velocityRotsPerSec;
  }

  /**
   * @return Torque urrent of roller
   */
  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }
}
