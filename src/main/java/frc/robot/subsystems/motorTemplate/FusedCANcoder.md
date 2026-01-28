



---MotorTemplateIO---
Replace
public boolean connected = false;
with
public boolean talonConnected = false;

Add the following to the inputs class
public boolean cancoderConnected = false;
public MagnetHealthValue magnetHealth = MagnetHealthValue.Magnet_Invalid;


---MotorTemplateIOReal---
Add all of the following
@Getter private static final double rotorToSensorReduction = 1;
@Getter private static final double sensorToMechanismReduction = 1;

// Hardware
private final CANcoder cancoder;

// Configuration
private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

// Status signals
private final StatusSignal<MagnetHealthValue> magnetHealth;

cancoder = new CANcoder(0, new CANBus("rio"));

// Feedback
talonConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
talonConfig.Feedback.RotorToSensorRatio = rotorToSensorReduction;
talonConfig.Feedback.SensorToMechanismRatio = sensorToMechanismReduction;

// CANcoder
cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
cancoderConfig.MagnetSensor.MagnetOffset = 0;
cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

// Configure status signals
magnetHealth = cancoder.getMagnetHealth();


---MotorTemplateIOSim---
replace
MotorTemplateIOReal.getReduction()
with
MotorTemplateIOReal.getReduction()


Replace
inputs.connected = true;
with
inputs.talonConnected = true;

inputs.cancoderConnected = true;
inputs.magnetHealth = MagnetHealthValue.Magnet_Green;
