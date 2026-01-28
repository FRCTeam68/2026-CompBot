package frc.robot.subsystems.shooter.hood;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import lombok.Getter;

public class HoodIOReal implements HoodIO {
  @Getter private static final double reduction = 1;

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

  // Control requests
  // TEMPLATE: Choose desired control methods
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final VelocityVoltage velocityOut = new VelocityVoltage(0).withEnableFOC(true);
  //   private final MotionMagicVelocityVoltage velocityOut = new
  // MotionMagicVelocityVoltage(0).withEnableFOC(true);
  //   private final VelocityTorqueCurrentFOC velocityOut = new VelocityTorqueCurrentFOC(0);
  //   private final MotionMagicVelocityTorqueCurrentFOC velocityOut = new
  // MotionMagicVelocityTorqueCurrentFOC(0);
  private final PositionVoltage positionOut = new PositionVoltage(0).withEnableFOC(true);
  //   private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0).withEnableFOC(true);
  //   private final TorqueCurrentFOC positionOut = new TorqueCurrentFOC(0);
  //   private final MotionMagicTorqueCurrentFOC positionOut = new MotionMagicTorqueCurrentFOC(0);
  private final NeutralOut neutralOut = new NeutralOut();

  public HoodIOReal() {
    // TEMPLATE: Set CAN id and bus
    talon = new TalonFX(0, new CANBus("rio"));

    // Configure Motor
    // TEMPLATE: Set configuration
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Current limits
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    // Feedback
    config.Feedback.SensorToMechanismRatio = reduction;
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
        // TEMPLATE: Set whether motor is attached to a CANivore
        new CANBus("rio"),
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius);
  }

  @Override
  public void updateInputs(MotorTemplateIOInputs inputs) {
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
  public void runVelocity(double velocity, int slot) {
    talon.setControl(velocityOut.withVelocity(velocity).withSlot(slot));
  }

  @Override
  public void runPosition(double position, int slot) {
    talon.setControl(positionOut.withPosition(position).withSlot(slot));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setPosition(double rotations) {
    talon.setPosition(rotations);
  }

  @Override
  public void setPID(SlotConfigs... newConfig) {
    for (int i = 0; i < Math.min(newConfig.length, 3); i++) {
      /*
       * TEMPLATE: Optionally add gravity type and static feedforward sign
       * Default gravity type: Elevator_Static
       * Default static feedforward sign: UseVelocitySign
       */
      SlotConfigs slotConfig = newConfig[i];
      switch (i) {
        case 0 -> config.Slot0 = Slot0Configs.from(slotConfig);
        case 1 -> config.Slot1 = Slot1Configs.from(slotConfig);
        case 2 -> config.Slot2 = Slot2Configs.from(slotConfig);
      }
    }
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setMotionMagic(MotionMagicConfigs newConfig) {
    tryUntilOk(5, () -> talon.getConfigurator().apply(newConfig, 0.25));
  }
}
