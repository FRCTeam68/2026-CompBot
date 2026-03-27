package frc.robot.subsystems.rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class RollerSystemIOSim implements RollerSystemIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;

  /**
   * @param motor The motor (or gearbox) attached to system.
   * @param reduction The ratio of motor to mechanism rotations, where a ratio greater than 1 is a
   *     reduction.
   * @param moi The moment of inertia of the roller. This can be roughly calculated from the CAD.
   *     Units are J/KgMetersSquared.
   */
  public RollerSystemIOSim(DCMotor motor, double reduction, double moi) {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moi, reduction), motor);
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0);
    }

    sim.update(Constants.loopPeriodSecs);

    inputs.connected = true;
    inputs.positionRots = sim.getAngularPositionRotations();
    inputs.velocityRotsPerSec = sim.getAngularVelocityRPM() / 60.0;
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps =
        (appliedVoltage > 0.0) ? sim.getCurrentDrawAmps() * 12.0 / appliedVoltage : 0.0;
  }

  @Override
  public void runVolts(double volts) {
    setInputVoltage(volts);
  }

  @Override
  public void runVelocity(double velocity) {
    sim.setAngularVelocity(velocity);
  }

  @Override
  public void stop() {
    runVolts(0);
  }

  private void setInputVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }
}
