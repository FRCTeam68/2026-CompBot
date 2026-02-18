package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil.ControlMode;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotor motor = DCMotor.getKrakenX60Foc(2);

  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(0, 0, 0);

  private SlotConfigs[] slotConfigs = new SlotConfigs[3];
  private ControlMode mode = ControlMode.Neutral;
  private double appliedVoltage = 0;

  public FlywheelIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, .1, FlywheelIOReal.getReduction()), motor);
  }

  @Override
  public void updateInputs(FlyWheelIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0);
    } else {
      if (mode == ControlMode.Velocity) {
        setInputVoltage(controller.calculate(sim.getAngularVelocityRPM() / 60.0));
      }
    }

    sim.update(Constants.loopPeriodSecs);

    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.positionRots = sim.getAngularPositionRotations();
    inputs.velocityRotsPerSec = sim.getAngularVelocityRPM() / 60.0;
    inputs.leaderAppliedVoltage = appliedVoltage;
    inputs.followerAppliedVoltage = -appliedVoltage;
    inputs.leaderSupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.leaderTorqueCurrentAmps =
        (appliedVoltage > 0.0) ? sim.getCurrentDrawAmps() * 12.0 / appliedVoltage : 0.0;
    ;
  }

  @Override
  public void runVolts(double volts) {
    mode = ControlMode.Voltage;
    setInputVoltage(volts);
  }

  @Override
  public void runVelocity(double velocity, int slot) {
    mode = ControlMode.Velocity;
    controller.setPID(slotConfigs[slot].kP, slotConfigs[slot].kI, slotConfigs[slot].kD);
    controller.setSetpoint(velocity);
  }

  @Override
  public void stop() {
    runVolts(0);
  }

  @Override
  public void setPID(SlotConfigs... newConfig) {
    for (int i = 0; i < Math.min(newConfig.length, 3); i++) {
      slotConfigs[i] = newConfig[i];
    }
  }

  private void setInputVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }
}
