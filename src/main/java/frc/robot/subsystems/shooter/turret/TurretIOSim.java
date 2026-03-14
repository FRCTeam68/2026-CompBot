package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil.ControlMode;

public class TurretIOSim implements TurretIO {
  private final DCMotor motor = DCMotor.getKrakenX44Foc(1);

  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(0, 0, 0);

  private SlotConfigs[] slotConfigs = new SlotConfigs[3];
  private ControlMode mode = ControlMode.Neutral;
  private double appliedVoltage = 0;

  public TurretIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, .1, TurretIOReal.getReduction()), motor);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    } else {
      if (mode == ControlMode.Position) {
        setInputVoltage(controller.calculate(sim.getAngularPositionRotations()));
      }
    }

    sim.update(Constants.loopPeriodSecs);

    inputs.motorConnected = true;
    inputs.cancoderConnected = true;
    inputs.positionDeg = Units.rotationsToDegrees(sim.getAngularPositionRotations());
    inputs.velocityDegPerSec = Units.rotationsToDegrees(sim.getAngularVelocityRPM() / 60.0);
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps =
        (appliedVoltage > 0.0) ? sim.getCurrentDrawAmps() * 12.0 / appliedVoltage : 0.0;
    inputs.magnetHealth = MagnetHealthValue.Magnet_Green;
    inputs.absolutePositionDeg =
        Units.rotationsToDegrees(MathUtil.inputModulus(sim.getAngularPositionRotations(), 0, 1));
  }

  @Override
  public void runVolts(double volts) {
    mode = ControlMode.Voltage;
    setInputVoltage(volts);
  }

  @Override
  public void runPosition(double degrees, int slot) {
    mode = ControlMode.Position;
    controller.setPID(slotConfigs[slot].kP, slotConfigs[slot].kI, slotConfigs[slot].kD);
    controller.setSetpoint(Units.degreesToRotations(degrees));
  }

  @Override
  public void stop() {
    runVolts(0);
  }

  @Override
  public void setPosition(double degrees) {
    sim.setAngle(Units.degreesToRadians(degrees));
  }

  @Override
  public void setPID(SlotConfigs... newConfig) {
    slotConfigs = newConfig;
  }

  private void setInputVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }
}
