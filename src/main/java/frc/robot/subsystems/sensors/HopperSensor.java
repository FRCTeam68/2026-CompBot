package frc.robot.subsystems.sensors;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.sensors.canrange.CANrangeIO;
import frc.robot.subsystems.sensors.canrange.CANrangeIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HopperSensor extends SubsystemBase {
  // Config
  @Getter private static final CANrangeConfiguration config = new CANrangeConfiguration();

  static {
    config.FovParams.FOVRangeX = 27;
    config.FovParams.FOVRangeY = 27;
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 3100;
    config.ProximityParams.ProximityHysteresis = 0.01;
    config.ProximityParams.ProximityThreshold = 0.4;
    config.ToFParams.UpdateMode = UpdateModeValue.ShortRangeUserFreq;
    config.ToFParams.UpdateFrequency = 50;
  }

  private final LoggedTunableNumber risingDebounceTime =
      new LoggedTunableNumber("HopperSensor/RisingDebounceTime", 0.62);
  private final LoggedTunableNumber fallingDebounceTime =
      new LoggedTunableNumber("HopperSensor/FallingDebounceTime", 0.5);

  // Alert
  private final Alert disconnectedAlert =
      new Alert("Hopper CANrange sensor disconnected.", AlertType.kWarning);

  // Debouncers
  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Debouncer detectedRisingDebouncer = new Debouncer(0.0, DebounceType.kRising);
  private final Debouncer detectedFallingDebouncer = new Debouncer(0.0, DebounceType.kFalling);

  private final CANrangeIO io;
  protected final CANrangeIOInputsAutoLogged inputs = new CANrangeIOInputsAutoLogged();

  public HopperSensor(CANrangeIO io) {
    this.io = io;

    Logger.recordOutput(
        "HopperSensor/ThresholdLow",
        config.ProximityParams.ProximityThreshold - config.ProximityParams.ProximityHysteresis,
        Meters);
    Logger.recordOutput(
        "HopperSensor/ThresholdHigh",
        config.ProximityParams.ProximityThreshold + config.ProximityParams.ProximityHysteresis,
        Meters);
  }

  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("HopperCANrange", inputs);
    disconnectedAlert.set(!connectedDebouncer.calculate(inputs.connected));

    // Update alerts
    if (risingDebounceTime.hasChanged(hashCode()) | fallingDebounceTime.hasChanged(hashCode())) {
      detectedRisingDebouncer.setDebounceTime(risingDebounceTime.get());
      detectedFallingDebouncer.setDebounceTime(fallingDebounceTime.get());
    }
  }

  /**
   * Returns true if the hopper is at least partially filled. The raw detected state is debounced to
   * provide a more stable signal when intaking.
   */
  @AutoLogOutput(key = "HopperSensor/IsNotEmpty")
  public boolean isNotEmpty() {
    return detectedRisingDebouncer.calculate(detectedFallingDebouncer.calculate(isDetectedRaw()));
  }

  /** Returns the raw detected signal from the CANrange. */
  public boolean isDetectedRaw() {
    return inputs.detected;
  }

  /**
   * Returns the distance to the nearest object the CANrange sees.
   *
   * <p><b>Units:<b\> Meters
   */
  public double getDistance() {
    return inputs.distanceMeters;
  }

  /** Returns if the CANrange is connected. */
  public boolean isConnected() {
    return inputs.connected;
  }
}
