package frc.robot.subsystems.intakePivot;

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

public class IntakePivotIOSim implements IntakePivotIO {
  private final DCMotor motor = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(0, 0, 0);

  private SlotConfigs[] slotConfigs = new SlotConfigs[3];
  private ControlMode mode = ControlMode.Neutral;
  private double appliedVoltage = 0;

  public IntakePivotIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, .1, IntakePivotIOReal.getReduction()), motor);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0);
    } else {
      if (mode == ControlMode.Position) {
        setInputVoltage(controller.calculate(sim.getAngularPositionRotations()));
      }
    }

    sim.update(Constants.loopPeriodSecs);
    if (sim.getAngularPositionRotations() > IntakePivot.getExtended()
        || sim.getAngularPositionRotations() < IntakePivot.getPackaged()) {
      sim.setAngle(
          Units.rotationsToRadians(
              MathUtil.clamp(
                  sim.getAngularPositionRotations(),
                  IntakePivot.getPackaged(),
                  IntakePivot.getExtended())));
    }

    inputs.motorConnected = true;
    inputs.cancoderConnected = true;
    inputs.positionRots = sim.getAngularPositionRotations();
    inputs.velocityRotsPerSec = sim.getAngularVelocityRPM() / 60.0;
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps =
        (appliedVoltage > 0.0) ? sim.getCurrentDrawAmps() * 12.0 / appliedVoltage : 0.0;
    inputs.magnetHealth = MagnetHealthValue.Magnet_Green;
    inputs.absolutePositionRots =
        MathUtil.inputModulus(
            sim.getAngularPositionRotations() / IntakePivotIOReal.getSensorToMechanismReduction(),
            0.0,
            1.0);
  }

  @Override
  public void runVolts(double volts) {
    mode = ControlMode.Voltage;
    setInputVoltage(volts);
  }

  @Override
  public void runPosition(double rotations, int slot) {
    mode = ControlMode.Position;
    controller.setPID(slotConfigs[slot].kP, slotConfigs[slot].kI, slotConfigs[slot].kD);
    controller.setSetpoint(
        MathUtil.clamp(rotations, IntakePivot.getPackaged(), IntakePivot.getExtended()));
  }

  @Override
  public void stop() {
    runVolts(0);
  }

  @Override
  public void setPosition(double rotations) {
    sim.setAngle(Units.rotationsToRadians(rotations));
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
