package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.LoggedTracerStatic;

/** Physics sim implementation of module IO. Simulation is always based on voltage control. */
public class ModuleIOSim implements ModuleIO {
  private static final DCMotor driveMotorModel = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor turnMotorModel = DCMotor.getKrakenX44Foc(1);

  private final DCMotorSim driveSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(driveMotorModel, 0.025, DriveConstants.driveReduction),
          driveMotorModel);
  private final DCMotorSim turnSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(turnMotorModel, 0.004, DriveConstants.turnReduction),
          turnMotorModel);

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(0.0, 0.0, 0.0);
  private PIDController turnController = new PIDController(0.0, 0.0, 0.0);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    LoggedTracerStatic.record("ModuleIOStart");
    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          MathUtil.clamp(
              driveController.calculate(driveSim.getAngularVelocityRadPerSec()), -12.0, 12.0);
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts =
          MathUtil.clamp(turnController.calculate(turnSim.getAngularPositionRad()), -12.0, 12.0);
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(driveAppliedVolts);
    turnSim.setInputVoltage(turnAppliedVolts);
    driveSim.update(Constants.loopPeriodSecs);
    turnSim.update(Constants.loopPeriodSecs);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnSupplyCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnEncoderMagnetHealth = MagnetHealthValue.Magnet_Green;

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void runDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void runTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void runDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void runTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }

  @Override
  public void setDrivePID(Slot0Configs newconfig) {
    driveController.setPID(newconfig.kP, newconfig.kI, newconfig.kD);
  }

  @Override
  public void setTurnPID(Slot0Configs newconfig) {
    turnController.setPID(newconfig.kP, newconfig.kI, newconfig.kD);
  }
}
