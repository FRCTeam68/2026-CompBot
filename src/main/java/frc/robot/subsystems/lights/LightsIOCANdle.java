package frc.robot.subsystems.lights;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.util.CanBusUtil;
import frc.robot.util.PhoenixUtil;

public class LightsIOCANdle implements LightsIO {
  private final CANBus canBus = CanBusUtil.getRioBus();

  // Hardware
  private final CANdle candle = new CANdle(60, canBus);

  // Config
  private final CANdleConfiguration config = new CANdleConfiguration();

  // Status signals
  private final StatusSignal<Current> outputCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public LightsIOCANdle() {
    // Configure hardware
    config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
    config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Off;
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;
    config.LED.BrightnessScalar = 1;
    config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
    config.LED.StripType = StripTypeValue.GRB;
    tryUntilOk(5, () -> candle.getConfigurator().apply(config, 0.25));

    // Configure status signals
    outputCurrent = candle.getOutputCurrent();
    tempCelsius = candle.getDeviceTemp();

    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, outputCurrent));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(candle));
    PhoenixUtil.registerSignals(canBus, outputCurrent, tempCelsius);

    // clear animation slots
    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount().getValue(); i++) {
      setControl(new EmptyAnimation(i));
    }
  }

  @Override
  public void updateInputs(LightsIOInputs inputs) {
    inputs.connected = BaseStatusSignal.isAllGood(outputCurrent);
    inputs.outputCurrent = outputCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void setBrightness(double percent) {
    config.LED.BrightnessScalar = percent;
    tryUntilOk(5, () -> candle.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setControl(ControlRequest request) {
    candle.setControl(request);
  }
}
