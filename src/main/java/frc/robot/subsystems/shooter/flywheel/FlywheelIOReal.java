package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.PhoenixUtil;
import lombok.Getter;

public class FlywheelIOReal implements FlywheelIO {
  @Getter private static final double reduction = 1;

  // Hardware
  private final TalonFX leaderTalon;
  private final TalonFX followerTalon;

  // Configuration
  private final TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration followerConfig = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> leaderAppliedVoltage;
  private final StatusSignal<Voltage> followerAppliedVoltage;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Current> leaderTorqueCurrent;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Temperature> leaderTempCelsius;
  private final StatusSignal<Temperature> followerTempCelsius;

  // Control requests
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC velocityOut = new VelocityTorqueCurrentFOC(0);
  private final NeutralOut neutralOut = new NeutralOut();

  public FlywheelIOReal() {
    leaderTalon = new TalonFX(25, ShooterConstants.canBus);
    followerTalon = new TalonFX(26, ShooterConstants.canBus);

    // Motor output
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Current limits
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.SupplyCurrentLimit = 80;
    leaderConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
    leaderConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    // Feedback
    leaderConfig.Feedback.SensorToMechanismRatio = reduction;
    tryUntilOk(5, () -> leaderTalon.getConfigurator().apply(leaderConfig, 0.25));

    // Follower
    followerConfig.MotorOutput.NeutralMode = leaderConfig.MotorOutput.NeutralMode;
    followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    followerConfig.CurrentLimits.SupplyCurrentLimit = leaderConfig.CurrentLimits.SupplyCurrentLimit;
    followerConfig.CurrentLimits.SupplyCurrentLowerTime =
        leaderConfig.CurrentLimits.SupplyCurrentLowerTime;
    followerConfig.CurrentLimits.SupplyCurrentLowerLimit =
        leaderConfig.CurrentLimits.SupplyCurrentLowerLimit;
    tryUntilOk(5, () -> leaderTalon.getConfigurator().apply(followerConfig, 0.25));
    followerTalon.setControl(
        new Follower(leaderTalon.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(100));

    position = leaderTalon.getPosition();
    velocity = leaderTalon.getVelocity();
    leaderAppliedVoltage = leaderTalon.getMotorVoltage();
    followerAppliedVoltage = followerTalon.getMotorVoltage();
    leaderSupplyCurrent = leaderTalon.getSupplyCurrent();
    followerSupplyCurrent = followerTalon.getSupplyCurrent();
    leaderTorqueCurrent = leaderTalon.getTorqueCurrent();
    followerTorqueCurrent = followerTalon.getTorqueCurrent();
    leaderTempCelsius = leaderTalon.getDeviceTemp();
    followerTempCelsius = followerTalon.getDeviceTemp();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100, leaderTorqueCurrent, followerTorqueCurrent));
    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                position,
                velocity,
                leaderAppliedVoltage,
                followerAppliedVoltage,
                leaderSupplyCurrent,
                followerSupplyCurrent));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(leaderTalon, followerTalon));
    PhoenixUtil.registerSignals(
        ShooterConstants.canBus,
        position,
        velocity,
        leaderAppliedVoltage,
        followerAppliedVoltage,
        leaderSupplyCurrent,
        leaderTorqueCurrent,
        leaderTempCelsius,
        followerTempCelsius);
  }

  @Override
  public void updateInputs(FlyWheelIOInputs inputs) {
    inputs.leaderConnected =
        BaseStatusSignal.isAllGood(
            position, velocity, leaderAppliedVoltage, leaderSupplyCurrent, leaderTorqueCurrent);
    inputs.followerConnected = BaseStatusSignal.isAllGood(followerAppliedVoltage);
    inputs.positionRots = position.getValueAsDouble();
    inputs.velocityRotsPerSec = velocity.getValueAsDouble();
    inputs.leaderAppliedVoltage = leaderAppliedVoltage.getValueAsDouble();
    inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrent.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.leaderTorqueCurrentAmps = leaderTorqueCurrent.getValueAsDouble();
    inputs.followerTorqueCurrentAmps = followerTorqueCurrent.getValueAsDouble();
    inputs.leaderTempCelsius = leaderTempCelsius.getValueAsDouble();
    inputs.followerTempCelsius = followerTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    leaderTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runVelocity(double velocity, int slot) {
    leaderTalon.setControl(velocityOut.withVelocity(velocity).withSlot(slot));
  }

  @Override
  public void stop() {
    leaderTalon.setControl(neutralOut);
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
          leaderConfig.Slot0 = Slot0Configs.from(newConfig[i]);
          break;
        case 1:
          leaderConfig.Slot1 = Slot1Configs.from(newConfig[i]);
          break;
        case 2:
          leaderConfig.Slot2 = Slot2Configs.from(newConfig[i]);
          break;
      }
    }
    tryUntilOk(5, () -> leaderTalon.getConfigurator().apply(leaderConfig, 0.25));
  }
}
