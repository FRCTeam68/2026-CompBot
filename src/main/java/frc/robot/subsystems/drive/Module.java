package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final LoggedTunableNumber drivekS =
      new LoggedTunableNumber("Drive/Module/DrivekS");
  private static final LoggedTunableNumber drivekV =
      new LoggedTunableNumber("Drive/Module/DrivekV");
  private static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("Drive/Module/DrivekP");
  private static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("Drive/Module/DrivekD");
  private static final LoggedTunableNumber turnkS = new LoggedTunableNumber("Drive/Module/TurnkS");
  private static final LoggedTunableNumber turnkP = new LoggedTunableNumber("Drive/Module/TurnkP");
  private static final LoggedTunableNumber turnkD = new LoggedTunableNumber("Drive/Module/TurnkD");

  static {
    switch (Constants.getMode()) {
      case REAL, REPLAY -> {
        drivekS.initDefault(0.01);
        drivekV.initDefault(0);
        drivekP.initDefault(2.0);
        drivekD.initDefault(0);
        turnkS.initDefault(2);
        turnkP.initDefault(50.0);
        turnkD.initDefault(0);
      }
      case SIM -> {
        drivekS.initDefault(0);
        drivekV.initDefault(0);
        drivekP.initDefault(0.1);
        drivekD.initDefault(0);
        turnkS.initDefault(0);
        turnkP.initDefault(10.0);
        turnkD.initDefault(0);
      }
    }
  }

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final String inputsKey;

  // Connected debouncers
  private final Debouncer driveMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer turnMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  // Alerts
  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private final Alert driveTempAlert;
  private final Alert turnTempAlert;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on drive " + DriveConstants.moduleNames[index] + "!",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on drive " + DriveConstants.moduleNames[index] + "!",
            AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on drive " + DriveConstants.moduleNames[index] + "!",
            AlertType.kError);
    driveTempAlert =
        new Alert(
            "Drive motor too hot on drive " + DriveConstants.moduleNames[index] + ".",
            AlertType.kWarning);
    turnTempAlert =
        new Alert(
            "Turn motor too hot on drive " + DriveConstants.moduleNames[index] + ".",
            AlertType.kWarning);
    inputsKey =
        "Drive/" + DriveConstants.moduleNames[index].replace(" ", "").replaceFirst("m", "M");

    io.setDrivePID(
        new Slot0Configs()
            .withKS(drivekS.get())
            .withKV(drivekV.get())
            .withKP(drivekP.get())
            .withKD(drivekD.get()));
    io.setTurnPID(
        new Slot0Configs().withKS(turnkS.get()).withKP(turnkP.get()).withKD(turnkD.get()));
  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs(inputsKey, inputs);
  }

  public void periodic() {
    if (Constants.tuningMode) {
      if (drivekS.hasChanged(hashCode())
          || drivekV.hasChanged(hashCode())
          || drivekP.hasChanged(hashCode())
          || drivekD.hasChanged(hashCode())) {
        io.setDrivePID(
            new Slot0Configs()
                .withKS(drivekS.get())
                .withKV(drivekV.get())
                .withKP(drivekP.get())
                .withKD(drivekD.get()));
      }
      if (turnkS.hasChanged(hashCode())
          || turnkP.hasChanged(hashCode())
          || turnkD.hasChanged(hashCode())) {
        io.setTurnPID(
            new Slot0Configs().withKS(turnkS.get()).withKP(turnkP.get()).withKD(turnkD.get()));
      }
    }

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * DriveConstants.wheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!driveMotorConnectedDebouncer.calculate(inputs.driveConnected));
    turnDisconnectedAlert.set(!turnMotorConnectedDebouncer.calculate(inputs.turnConnected));
    turnEncoderDisconnectedAlert.set(
        !turnEncoderConnectedDebouncer.calculate(inputs.turnEncoderConnected));
    driveTempAlert.set(inputs.driveTempCelsius > Constants.warningTempCelsius);
    turnTempAlert.set(inputs.turnTempCelsius > Constants.warningTempCelsius);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    io.runDriveVelocity(state.speedMetersPerSecond / DriveConstants.wheelRadius);
    io.runTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.runDriveOpenLoop(output);
    io.runTurnPosition(new Rotation2d());
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.runDriveOpenLoop(0.0);
    io.runTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.wheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.wheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
