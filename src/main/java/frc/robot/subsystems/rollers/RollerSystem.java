package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.Slot0Configs;
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
import org.littletonrobotics.junction.Logger;

public class RollerSystem extends SubsystemBase {
  private final String name;
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();

  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert disconnectedAlert;
  private final Alert tempAlert;

  private LoggedTunableNumber kP;
  private LoggedTunableNumber kD;
  private LoggedTunableNumber kS;

  @Getter private double setpoint = 0.0;

  @Getter private ControlMode mode = ControlMode.Neutral;

  public RollerSystem(String name, RollerSystemIO io) {
    this.name = name;
    this.io = io;

    kP = new LoggedTunableNumber(name + "/kP");
    kD = new LoggedTunableNumber(name + "/kD");
    kS = new LoggedTunableNumber(name + "/kS");

    disconnectedAlert = new Alert(name + " motor disconnected!", AlertType.kError);
    tempAlert = new Alert(name + " motor is too hot.", AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnectedAlert.set(!connectedDebouncer.calculate(inputs.connected));
    tempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);

    Logger.recordOutput(name + "/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput(
        name + "/SetpointRotsPerSec", (mode == ControlMode.Velocity) ? setpoint : 0);
    Logger.recordOutput(
        name + "/SetpointPositionRots", (mode == ControlMode.Position) ? setpoint : 0);

    // TODO: do we always check this. we wouldn't need to call set pid in rollersystem
    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode()) || kS.hasChanged(hashCode())) {
      io.setPID(new Slot0Configs().withKP(kP.get()).withKD(kD.get()).withKS(kS.get()));
    }
  }

  /** Must call this once and only once in robotcontainer after each RollerSystem is created */
  public void initPID(SlotConfigs newConfig) {
    kP.initDefault(newConfig.kP);
    kD.initDefault(newConfig.kD);
    kS.initDefault(newConfig.kS);
  }

  /** Run roller at volts */
  public void runVolts(double inputVolts) {
    setpoint = inputVolts;
    mode = ControlMode.Voltage;
    io.runVolts(inputVolts);
  }

  /**
   * Run roller at velocity
   *
   * @param velocity Velocity in mechanism rotations per second
   */
  public void runVelocity(double velocity) {
    setpoint = velocity;
    mode = ControlMode.Velocity;
    io.runVelocity(velocity);
  }

  /**
   * Run roller to position
   *
   * @param rotations Position in mechanism rotations
   */
  public void runPosition(double position) {
    setpoint = position;
    mode = ControlMode.Position;
    io.runPosition(position);
  }

  /** Stop roller */
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
   * @return Velocity of roller in mechanism rotations per second
   */
  public double getVelocity() {
    return inputs.velocityRotsPerSec;
  }

  /**
   * @return Position of roller in mechanism rotations
   */
  public double getPosition() {
    return inputs.positionRots;
  }

  /**
   * @return TorqueCurrent of roller
   */
  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }
}
