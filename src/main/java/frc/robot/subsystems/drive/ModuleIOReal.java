package frc.robot.subsystems.drive;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;
import frc.robot.util.PhoenixUtil;
import java.util.Queue;

public class ModuleIOReal implements ModuleIO {
  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Configurations
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

  // Control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);

  @SuppressWarnings("unused")
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);

  @SuppressWarnings("unused")
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage velocityVoltageRequest =
      new VelocityVoltage(0.0).withEnableFOC(true);

  // Timestamp inputs from Phoenix thread
  private final Queue<Double> timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveSupplyCurrent;
  private final StatusSignal<Current> driveTorqueCurrent;
  private final StatusSignal<Temperature> driveTempCelsius;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<MagnetHealthValue> turnMagnetHealth;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnSupplyCurrent;
  private final StatusSignal<Current> turnTorqueCurrent;
  private final StatusSignal<Temperature> turnTempCelsius;

  // private final StatusSignal<Boolean> turnEncoderSyncStickyFault;

  public ModuleIOReal(ModuleConfig constants) {
    driveTalon = new TalonFX(constants.driveMotorId(), DriveConstants.canBus);
    turnTalon = new TalonFX(constants.turnMotorId(), DriveConstants.canBus);
    cancoder = new CANcoder(constants.encoderId(), DriveConstants.canBus);

    // Configure drive motor
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveReduction;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = DriveConstants.driveStatorCurrentLimitAmps;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -DriveConstants.driveStatorCurrentLimitAmps;
    driveConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.driveStatorCurrentLimitAmps;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.encoderId();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.RotorToSensorRatio = DriveConstants.turnReduction;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = DriveConstants.turnStatorCurrentLimitAmps;
    turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -DriveConstants.turnStatorCurrentLimitAmps;
    turnConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.turnStatorCurrentLimitAmps;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.MotorOutput.Inverted =
        constants.turnInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    cancoderConfig.MagnetSensor.MagnetOffset = constants.encoderOffset().getRotations();
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.encoderInverted()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

    // Create timestamp queue
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition().clone());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();
    driveTempCelsius = driveTalon.getDeviceTemp();

    // Create turn status signals
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition().clone());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnSupplyCurrent = turnTalon.getSupplyCurrent();
    turnTorqueCurrent = turnTalon.getTorqueCurrent();
    turnTempCelsius = turnTalon.getDeviceTemp();
    // turnEncoderSyncStickyFault = turnTalon.getStickyFault_FusedSensorOutOfSync();

    // Create encoder status signals
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnMagnetHealth = cancoder.getMagnetHealth();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.odometryFrequency, drivePosition, turnPosition, turnAbsolutePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveTorqueCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrent,
        turnTorqueCurrent,
        // turnEncoderSyncStickyFault,
        turnMagnetHealth);
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon, cancoder));
    PhoenixUtil.registerSignals(
        DriveConstants.canBus,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveTorqueCurrent,
        driveTempCelsius,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrent,
        turnTorqueCurrent,
        turnTempCelsius,
        // turnEncoderSyncStickyFault,
        turnAbsolutePosition,
        turnMagnetHealth);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive motor inputs
    inputs.driveConnected =
        BaseStatusSignal.isAllGood(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveSupplyCurrent,
            driveTorqueCurrent);
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();
    inputs.driveTempCelsius = driveTempCelsius.getValueAsDouble();

    // Update turn motor inputs
    inputs.turnConnected =
        BaseStatusSignal.isAllGood(
            turnPosition, turnVelocity, turnAppliedVolts, turnSupplyCurrent, turnTorqueCurrent);
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();
    inputs.turnTorqueCurrentAmps = turnTorqueCurrent.getValueAsDouble();
    inputs.turnTempCelsius = turnTempCelsius.getValueAsDouble();
    // inputs.turnEncoderSyncStickyFault = turnEncoderSyncStickyFault.getValue();

    // Update turn encoder inputs
    inputs.turnEncoderConnected =
        BaseStatusSignal.isAllGood(turnAbsolutePosition, turnMagnetHealth);
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnEncoderMagnetHealth = turnMagnetHealth.getValue();

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void runDriveOpenLoop(double output) {
    driveTalon.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void runTurnOpenLoop(double output) {
    turnTalon.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void runDriveVelocity(double velocityRadPerSec) {
    driveTalon.setControl(
        velocityVoltageRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void runTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
  }

  @Override
  public void setDrivePID(Slot0Configs config) {
    driveConfig.Slot0 = config;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
  }

  @Override
  public void setTurnPID(Slot0Configs config) {
    turnConfig.Slot0 = config;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    driveConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
  }
}
