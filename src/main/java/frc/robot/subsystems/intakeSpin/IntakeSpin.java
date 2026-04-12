package frc.robot.subsystems.intakeSpin;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualPD;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class IntakeSpin extends SubsystemBase {
  // Alerts
  private final Alert leaderDisconnectedAlert =
      new Alert("Intake Spin leader (left) motor disconnected!", AlertType.kError);
  private final Alert followerDisconnectedAlert =
      new Alert("Intake Spin follower (right) motor disconnected!", AlertType.kError);
  private final Alert leaderTempAlert =
      new Alert("Intake Spin leader (left) motor is too hot.", AlertType.kWarning);
  private final Alert followerTempAlert =
      new Alert("Intake Spin follower (right) motor is too hot.", AlertType.kWarning);
  private final Alert leaderRotorFaultAlert =
      new Alert(
          "Intake Spin leader (left) motor has a rotor fault. The motor may not run properly.",
          AlertType.kError);
  private final Alert followerRotorFaultAlert =
      new Alert(
          "Intake Spin leader (left) motor has a rotor fault. The motor may not run properly.",
          AlertType.kError);

  // Debouncers
  private final Debouncer leaderConnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);

  // PID tunables for velocity control (slot 0)
  private final LoggedTunableNumber kP = new LoggedTunableNumber("IntakeSpin/Slot0/kP", 0.5);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("IntakeSpin/Slot0/kD", 0.0);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("IntakeSpin/kS", 0.0);

  private final IntakeSpinIO io;
  protected final IntakeSpinIOInputsAutoLogged inputs = new IntakeSpinIOInputsAutoLogged();
  @Getter private double setpointVolts = 0.0;
  @Getter private double setpointRps = 0.0;

  public IntakeSpin(IntakeSpinIO io) {
    this.io = io;

    VirtualPD.registerMotor(
        () -> Watts.of(Math.abs(inputs.leaderSupplyCurrentAmps * inputs.leaderAppliedVoltage)),
        "IntakeSpin");
    VirtualPD.registerMotor(
        () -> Watts.of(Math.abs(inputs.followerSupplyCurrentAmps * inputs.followerAppliedVoltage)),
        "IntakeSpin");
  }

  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("IntakeSpin", inputs);

    // Update alerts
    leaderDisconnectedAlert.set(!leaderConnectedDebouncer.calculate(inputs.leaderConnected));
    followerDisconnectedAlert.set(!followerConnectedDebouncer.calculate(inputs.followerConnected));
    leaderTempAlert.set(inputs.leaderTempCelsius > Constants.warningTempCelsius);
    followerTempAlert.set(inputs.followerTempCelsius > Constants.warningTempCelsius);
    leaderRotorFaultAlert.set(inputs.leaderFaultRotorFault1 || inputs.leaderFaultRotorFault2);
    followerRotorFaultAlert.set(inputs.followerFaultRotorFault1 || inputs.followerFaultRotorFault2);

    // Update PID gains if changed
    if (kP.hasChanged(hashCode()) | kD.hasChanged(hashCode()) | kS.hasChanged(hashCode())) {
      io.setPID(new SlotConfigs().withKP(kP.get()).withKD(kD.get()).withKS(kS.get()));
    }
  }

  /**
   * Run system at specified voltage.
   *
   * @param volts Voltage to run the motor at.
   */
  public void runVolts(double volts) {
    setpointVolts = volts;
    io.runVolts(setpointVolts);
    Logger.recordOutput("IntakeSpin/SetpointVolts", setpointVolts, Volts);
  }

  /**
   * Run system to specified velocity in rotations per second.
   *
   * @param rps rotations per second
   */
  public void runVelocity(double rps) {
    setpointRps = rps;
    io.runVelocity(setpointRps, 0);
    Logger.recordOutput("IntakeSpin/SetpointRPS", setpointRps);
  }

  /** Stop motor with neutral output. */
  public void stop() {
    setpointVolts = 0.0;
    io.stop();
    Logger.recordOutput("IntakeSpin/SetpointVolts", 0.0, Volts);
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
    return inputs.leaderTorqueCurrentAmps;
  }

  /** Configure dashboard tuning controls for manual control. */
  public void configureDashboardControls() {
    SmartDashboard.putNumber("IntakeSpin/RPS", 0.0);
    SmartDashboard.putData(
        "IntakeSpin/RunRPS",
        Commands.runOnce(() -> runVelocity(SmartDashboard.getNumber("IntakeSpin/RPS", 0.0)), this)
            .withName("Dashboard/IntakeSpin/RunRPS"));
  }
}
