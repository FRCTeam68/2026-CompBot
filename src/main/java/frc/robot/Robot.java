package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.auton.Auton;
import frc.robot.util.CanBusUtil;
import frc.robot.util.ElasticUtil;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LoggedTracer;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final double lowBatteryDisabledVoltage = 11.0;
  private static final double lowBatteryEnabledVoltage = 9.0;
  private static final double lowBatteryDisabledTime = 2.0;

  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private final Timer disabledTimer = new Timer();

  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, turn off the robot or replace the battery to avoid damage.",
          AlertType.kWarning);

  @SuppressWarnings("unused")
  public Robot() {
    // Record metadata
    Logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    Logger.recordMetadata("LowCeiling", Boolean.toString(Constants.lowCeiling));
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });
    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        String inPath = LogFileUtil.findReplayLog();
        String outPath = LogFileUtil.addPathSuffix(inPath, "_sim");
        Logger.setReplaySource(new WPILOGReader(inPath));
        Logger.addDataReceiver(new WPILOGWriter(outPath));
        break;
    }

    // Set AdvantageKit timing mode
    setUseTiming(Constants.getMode() != frc.robot.Constants.Mode.REPLAY);

    // CTRE Hoot logging
    if (Constants.hootLogging && Constants.getMode() == Mode.REAL) {
      SignalLogger.setPath("//media/sda1/logs");
      SignalLogger.start();
    } else {
      SignalLogger.enableAutoLogging(false);
    }

    // Start AdvantageKit logger
    Logger.start();

    // Adjust loop overrun warning timeout
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(Constants.loopOverrunWarningSecs);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(Constants.loopOverrunWarningSecs);

    // Rely on our custom alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    // Configure DriverStation for sim
    if (Constants.getMode() == frc.robot.Constants.Mode.SIM) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }

    // Configure brownout voltage
    // This only does anything on the roboRIO 2. On the roboRIO 1 it is a no-op.
    RobotController.setBrownoutVoltage(6.0);

    // Reset alert timers
    disabledTimer.restart();

    // Set up auto logging
    AutoLogOutputManager.addObject(new HubShiftUtil());
    AutoLogOutputManager.addObject(new ShooterCommands());

    // Instantiate our RobotContainer
    robotContainer = new RobotContainer();

    // Warmup pathplanner libraries
    CommandScheduler.getInstance()
        .schedule(FollowPathCommand.warmupCommand().withName("PathplannerFollowPathWarmup"));
    // Uncomment the warmup command below if using pathplanner pathfinding
    // CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand().withName("PathplannerPathfindingWarmup"));
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop timing
    // Threads.setCurrentThreadPriority(true, 99);

    // Update shift conditions
    HubShiftUtil.update();

    // Refresh all Phoenix signals
    LoggedTracer.reset();
    PhoenixUtil.refreshAll();
    LoggedTracer.record("PhoenixRefresh");

    CommandScheduler.getInstance().run();
    LoggedTracer.record("CommandScheduler");

    // Return to non-RT thread priority
    // Threads.setCurrentThreadPriority(false, 10);

    // Low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if ((RobotController.getBatteryVoltage() > 0.0
                && (RobotController.getBatteryVoltage() <= lowBatteryEnabledVoltage)
            || (RobotController.getBatteryVoltage() <= lowBatteryDisabledVoltage
                && disabledTimer.hasElapsed(lowBatteryDisabledTime)))
        || lowBatteryAlert.get() == true) {
      lowBatteryAlert.set(true);
    }

    // Robot container periodic method
    robotContainer.updateAlerts();

    // Log robot visualization
    robotContainer.visualizeRobot();

    // Log status of CAN buses
    CanBusUtil.logStatus();

    // Record cycle time
    LoggedTracer.record("RobotPeriodic");
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    robotContainer.stopSubsystems();

    // Save Limelight 4 rewind when the robot disables at the end of a real match.
    if (DriverStation.isFMSAttached()
        && !DriverStation.isAutonomous()
        && DriverStation.getMatchTime() == 0) robotContainer.saveLimelightRewind();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    Auton.UpdateAlerts();
  }

  /** This function is called once when the robot is enabled in any mode. */
  @Override
  public void disabledExit() {
    // This must be done here to reset time for repeated practice matches
    HubShiftUtil.seedMatchTime();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = Auton.SelectedCommand();
    if (autonomousCommand != null) {
      Auton.setStartingPose();
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    if (DriverStation.isFMSAttached()) {
      if (AllianceFlipUtil.shouldFlip()) {
        ElasticUtil.selectTab("Teleop-Red");
      } else {
        ElasticUtil.selectTab("Teleop-Blue");
      }
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    ShooterCommands.clearStaticSetpoint();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    robotContainer.configureTestModeCommands();

    robotContainer.enableMT1();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
