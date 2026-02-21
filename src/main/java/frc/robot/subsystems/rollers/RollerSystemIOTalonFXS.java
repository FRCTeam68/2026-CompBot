package frc.robot.subsystems.rollers;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Generic roller IO implementation for a roller or series of rollers using a TalonFXS. */
public class RollerSystemIOTalonFXS implements RollerSystemIO {
  // Hardware
  private final TalonFXS talon;

  // Configuration
  private final TalonFXSConfiguration config = new TalonFXSConfiguration();

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  // Control requests
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
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
  public RollerSystemIOTalonFXS(
      int id,
      CANBus canBus,
      int currentLimitAmps,
      InvertedValue invertedValue,
      NeutralModeValue neutralModeValue,
      double reduction,
      MotorArrangementValue motor) {
    talon = new TalonFXS(id, canBus);

    // Communication
    config.Commutation.MotorArrangement = motor;
    config.Commutation.AdvancedHallSupport =
        (motor == MotorArrangementValue.Brushed_DC)
            ? AdvancedHallSupportValue.Disabled
            : AdvancedHallSupportValue.Enabled;
    // Motor output
    config.MotorOutput.Inverted = invertedValue;
    config.MotorOutput.NeutralMode = neutralModeValue;
    // Current limits
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    // Feedback
    config.ExternalFeedback.SensorToMechanismRatio = reduction;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon));
    PhoenixUtil.registerSignals(
        canBus, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);
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
