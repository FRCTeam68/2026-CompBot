package frc.robot.subsystems.intakeSpin;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeSpinIOSim implements IntakeSpinIO {
  private final DCMotor motor = DCMotor.getKrakenX60Foc(2);

  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;

  /**
   * @param motor The motor (or gearbox) attached to system.
   * @param reduction The ratio of motor to mechanism rotations, where a ratio greater than 1 is a
   *     reduction.
   * @param moi The moment of inertia of the roller. This can be roughly calculated from the CAD.
   *     Units are J/KgMetersSquared.
   */
  public IntakeSpinIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.002, IntakeSpinIOReal.getReduction()),
            motor);
  }

  @Override
  public void updateInputs(IntakeSpinIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0);
    }

    sim.update(Constants.loopPeriodSecs);

    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.positionRots = sim.getAngularPositionRotations();
    inputs.velocityRotsPerSec = sim.getAngularVelocityRPM() / 60.0;
    inputs.leaderAppliedVoltage = appliedVoltage;
    inputs.followerAppliedVoltage = -appliedVoltage;
    inputs.leaderSupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.followerSupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.leaderTorqueCurrentAmps =
        (appliedVoltage > 0.0) ? sim.getCurrentDrawAmps() * 12.0 / appliedVoltage : 0.0;
    inputs.leaderTorqueCurrentAmps =
        (appliedVoltage > 0.0) ? sim.getCurrentDrawAmps() * 12.0 / appliedVoltage : 0.0;
  }

  @Override
  public void runVolts(double volts) {
    setInputVoltage(volts);
  }

  @Override
  public void runVelocity(double velocity, int slot) {
    // Simple simulation approximation: map desired rps to a voltage proportional to a nominal
    // speed.
    // Choose 100 rps as the nominal full-speed (12V) value unless tuned otherwise.
    double nominalRPS = 100.0;
    double volts = (velocity / nominalRPS) * 12.0;
    setInputVoltage(volts);
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
