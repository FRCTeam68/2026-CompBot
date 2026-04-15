package frc.robot.subsystems.rollers;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Generic roller IO implementation for a roller or series of rollers using a TalonFX. */
public class RollerSystemIOTalonFX implements RollerSystemIO {
  // Hardware
  private final TalonFX talon;

  // Configuration
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<Boolean> faultRotorFault1;
  private final StatusSignal<Boolean> faultRotorFault2;

  // Control requests
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final NeutralOut neutralOut = new NeutralOut();

  /**
   * @param id CAN id of motor.
   * @param canBus CAN bus this device is on.
   * @param currentLimitAmps Max supply current. Supply current is limited to 40 amps after 1
   *     second.
   * @param invertedValue Positive direction of the motor when looking at the face of the motor.
   * @param neutralModeValue Neutral mode of the motor (Brake/Coast).
   * @param reduction The ratio of motor to mechanism rotations, where a ratio greater than 1 is a
   *     reduction.
   */
  public RollerSystemIOTalonFX(
      int id,
      CANBus canBus,
      int currentLimitAmps,
      InvertedValue invertedValue,
      NeutralModeValue neutralModeValue,
      double reduction) {
    talon = new TalonFX(id, canBus);

    // Motor output
    config.MotorOutput.Inverted = invertedValue;
    config.MotorOutput.NeutralMode = neutralModeValue;
    // Current limits
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 100;
    config.TorqueCurrent.PeakReverseTorqueCurrent = 100;
    // Feedback
    config.Feedback.SensorToMechanismRatio = reduction;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();
    faultRotorFault1 = talon.getFault_RotorFault1();
    faultRotorFault2 = talon.getFault_RotorFault2();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                faultRotorFault1,
                faultRotorFault2));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon));
    PhoenixUtil.registerSignals(
        canBus,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        faultRotorFault1,
        faultRotorFault2);
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.isAllGood(
            position, velocity, appliedVoltage, supplyCurrent, torqueCurrent);
    inputs.positionRots = position.getValueAsDouble();
    inputs.velocityRotsPerSec = velocity.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.faultRotorFault1 = faultRotorFault1.getValue();
    inputs.faultRotorFault2 = faultRotorFault2.getValue();
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }
}
