package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil.ControlMode;

public class RollerSystemIOSim implements RollerSystemIO {
  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(0, 0, 0);

  private Slot0Configs slotConfig = new Slot0Configs();
  private ControlMode mode = ControlMode.Neutral;
  private double appliedVoltage = 0.0;

  /**
   * @param motor The motor (or gearbox) attached to system.
   * @param reduction The ratio of motor to mechanism rotations, where a ratio greater than 1 is a
   *     reduction.
   * @param moi The moment of inertia of the roller. This can be roughly calculated from the CAD.
   *     Units are in JKgMetersSquared.
   */
  public RollerSystemIOSim(DCMotor motor, double reduction, double moi) {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moi, reduction), motor);
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0);
    } else {
      if (mode == ControlMode.Velocity) {
        setInputVoltage(controller.calculate(sim.getAngularVelocityRPM() / 60.0));
      } else if (mode == ControlMode.Position) {
        setInputVoltage(controller.calculate(sim.getAngularPositionRotations()));
      }
    }

    sim.update(Constants.loopPeriodSecs);

    inputs.connected = true;
    inputs.positionRots = sim.getAngularPositionRotations();
    inputs.velocityRotsPerSec = sim.getAngularVelocityRPM() / 60.0;
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps() * 12.0 / appliedVoltage;
  }

  @Override
  public void runVolts(double volts) {
    mode = ControlMode.Voltage;
    setInputVoltage(volts);
  }

  @Override
  public void runVelocity(double velocity) {
    mode = ControlMode.Velocity;
    controller.setPID(slotConfig.kP, slotConfig.kI, slotConfig.kD);
    controller.setSetpoint(velocity);
  }

  @Override
  public void runPosition(double position) {
    mode = ControlMode.Position;
    controller.setPID(slotConfig.kP, slotConfig.kI, slotConfig.kD);
    controller.setSetpoint(position);
  }

  @Override
  public void stop() {
    mode = ControlMode.Neutral;
    runVolts(0);
  }

  @Override
  public void setPosition(double rotations) {
    sim.setAngle(Units.rotationsToRadians(rotations));
  }

  @Override
  public void setPID(Slot0Configs newConfig) {
    slotConfig = newConfig;
  }

  private void setInputVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }
}
