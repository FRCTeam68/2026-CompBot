package frc.robot.subsystems.shooter.turret;

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

public class TurretIOReal implements TurretIO {
  private static final double rotorToSensorReduction = (50.0 / 8.0) * (62.0 / 18.0) * (52.0 / 26.0);
  private static final double sensorToMechanismReduction =
      (26.0 / 52.0) * (18.0 / 62.0) * (124.0 / 18.0);

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

  public TurretIOReal() {
    talon = new TalonFX(29, ShooterConstants.canBus);
    cancoder = new CANcoder(43, ShooterConstants.canBus);

    // Configure Motor
    talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
    // Motion limits
    talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(Turret.getMaximum());
    talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(Turret.getMinimum());
    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));

    // CANcoder
    cancoderConfig.MagnetSensor.MagnetOffset = -0.9677734375;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
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
  public void updateInputs(TurretIOInputs inputs) {
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
    inputs.absolutePositionDeg = Units.rotationsToDegrees(absolutePosition.getValueAsDouble());
    inputs.magnetHealth = magnetHealth.getValue();
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runPosition(double degrees, int slot) {
    talon.setControl(positionOut.withPosition(Units.degreesToRotations(degrees)).withSlot(slot));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setPosition(double degrees) {
    cancoder.setPosition(Units.degreesToRotations(degrees));
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
