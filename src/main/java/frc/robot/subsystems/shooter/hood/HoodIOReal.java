package frc.robot.subsystems.shooter.hood;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.PhoenixUtil;
import lombok.Getter;

public class HoodIOReal implements HoodIO {
  @Getter private static final double rotorToSensorReduction = (48.0 / 12.0) * (16.0 / 40.0);
  @Getter private static final double sensorToMechanismReduction = (295.0 / 30.0);

  @Getter
  private static final double reduction = rotorToSensorReduction * sensorToMechanismReduction;

  // Hardware
  private final TalonFX talon;
  private final CANcoder cancoder;

  // Configuration
  private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<MagnetHealthValue> magnetHealth;
  private final StatusSignal<Angle> absolutePosition;

  // Control requests
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  // private final PositionVoltage positionOut = new PositionVoltage(0).withEnableFOC(true);
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0).withEnableFOC(true);
  //   private final TorqueCurrentFOC positionOut = new TorqueCurrentFOC(0);
  //   private final MotionMagicTorqueCurrentFOC positionOut = new MotionMagicTorqueCurrentFOC(0);
  private final NeutralOut neutralOut = new NeutralOut();

  public HoodIOReal() {
    talon = new TalonFX(27, ShooterConstants.canBus);
    cancoder = new CANcoder(28, ShooterConstants.canBus);

    // Motor output
    talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Current limits
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfig.CurrentLimits.SupplyCurrentLimit = 80;
    talonConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
    talonConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    // Feedback
    talonConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    talonConfig.Feedback.RotorToSensorRatio = rotorToSensorReduction;
    talonConfig.Feedback.SensorToMechanismRatio = sensorToMechanismReduction;
    // Motion Limits
    talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
    talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Hood.getMinimum() - Hood.getMaximum();
    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));

    // CANcoder
    cancoderConfig.MagnetSensor.MagnetOffset = -0.097900390625; // Minimum elevation
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.1;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();
    magnetHealth = cancoder.getMagnetHealth();
    absolutePosition = cancoder.getAbsolutePosition();

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
                magnetHealth,
                absolutePosition));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon, cancoder));
    PhoenixUtil.registerSignals(
        ShooterConstants.canBus,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        magnetHealth,
        absolutePosition);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.isAllGood(
            position, velocity, appliedVoltage, supplyCurrent, torqueCurrent);
    inputs.cancoderConnected = BaseStatusSignal.isAllGood(magnetHealth, absolutePosition);
    inputs.positionDeg = Units.rotationsToDegrees(position.getValueAsDouble());
    inputs.velocityDegPerSec = Units.rotationsToDegrees(velocity.getValueAsDouble());
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.magnetHealth = magnetHealth.getValue();
    inputs.absolutePositionDeg = Units.rotationsToDegrees(absolutePosition.getValueAsDouble());
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runPosition(double elevation, int slot) {
    talon.setControl(positionOut.withPosition(Units.degreesToRotations(elevation)).withSlot(slot));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setPosition(double elevation) {
    cancoder.setPosition(Units.degreesToRotations(elevation));
  }

  @Override
  public void setPID(SlotConfigs... newConfig) {
    for (int i = 0; i < Math.min(newConfig.length, 3); i++) {
      /*
      Optionally add gravity type and static feedforward sign.
      Default gravity type: Elevator_Static
      Default static feedforward sign: UseVelocitySign
      */
      switch (i) {
        case 0:
          talonConfig.Slot0 = Slot0Configs.from(newConfig[i]);
          break;
        case 1:
          talonConfig.Slot1 = Slot1Configs.from(newConfig[i]);
          break;
        case 2:
          talonConfig.Slot2 = Slot2Configs.from(newConfig[i]);
          break;
      }
    }
    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
  }

  @Override
  public void setMotionMagic(MotionMagicConfigs newConfig) {
    talonConfig.MotionMagic = newConfig;
    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
  }
}
