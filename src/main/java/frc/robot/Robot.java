package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.CanbusReader;
import frc.robot.util.LoggedTracer;
import frc.robot.util.PhoenixUtil;
import java.lang.reflect.Field;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
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
  // TODO: consider additional low bat alarms. see MA
  private static final double loopOverrunWarningTimeout = 0.2; // seconds
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final double rioErrorTimeThreshold = 0.5; // Seconds to disable alert

  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer rioErrorTimer = new Timer();
  private final CanbusReader rioReader = new CanbusReader(new CANBus("rio"));

  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert rioErrorAlert =
      new Alert("CANivore errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert jitAlert =
      new Alert("Please wait to enable, JITing in progress.", AlertType.kWarning);

  public Robot() {
    // Record metadata
    Logger.recordMetadata("Robot", "2024");
    Logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    // TODO: MA is logging using RLOGServer instead of NT4Publisher. What is better?
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        LoggedPowerDistribution.getInstance(35, ModuleType.kRev);
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
        break;
    }

    // CTRE Hoot logging
    // do not call the setPath and hoot log will be logged to rio at "/home/lvuser/logs"
    SignalLogger.enableAutoLogging(false);
    // SignalLogger.setPath("//media/sda1/logs");
    // SignalLogger.start();

    // Set up auto logging for RobotState
    AutoLogOutputManager.addObject(new RobotState());

    // Start AdvantageKit logger
    Logger.start();

    // Adjust loop overrun warning timeout
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningTimeout);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);

    // Rely on our custom alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure DriverStation for sim
    if (Constants.getMode() == frc.robot.Constants.Mode.SIM) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
    }

    // Configure brownout voltage
    RobotController.setBrownoutVoltage(6.0);

    // Reset alert timers
    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    rioErrorTimer.restart();

    // Instantiate our RobotContainer. This will perform all our button bindings.
    robotContainer = new RobotContainer();

    // Warmup pathplanner libraries
    // This must be done after instantiate RobotContainer
    // TODO: These are 2 different commands. Do we need both?
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // TODO: Learn more about thead priority. MA has it always set to 1.
    // Threads.setCurrentThreadPriority(true, 99);

    // Refresh all Phoenix signals
    LoggedTracer.reset();
    PhoenixUtil.refreshAll();
    LoggedTracer.record("PhoenixRefresh");

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();
    LoggedTracer.record("CommandScheduler");

    // Return to non-RT thread priority
    // Threads.setCurrentThreadPriority(false, 10);

    // Robot container periodic method
    robotContainer.updateAlerts();

    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

    // Log CANivore status
    if (Constants.getMode() == Constants.Mode.REAL) {
      var rioStatus = rioReader.getStatus();
      if (rioStatus.isPresent()) {
        Logger.recordOutput("CANivoreStatus/Status", rioStatus.get().Status.getName());
        Logger.recordOutput("CANivoreStatus/Utilization", rioStatus.get().BusUtilization);
        Logger.recordOutput("CANivoreStatus/OffCount", rioStatus.get().BusOffCount);
        Logger.recordOutput("CANivoreStatus/TxFullCount", rioStatus.get().TxFullCount);
        Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", rioStatus.get().REC);
        Logger.recordOutput("CANivoreStatus/TransmitErrorCount", rioStatus.get().TEC);
        if (!rioStatus.get().Status.isOK()
            || canStatus.transmitErrorCount > 0
            || canStatus.receiveErrorCount > 0) {
          rioErrorTimer.restart();
        }
      }
      rioErrorAlert.set(
          !rioErrorTimer.hasElapsed(rioErrorTimeThreshold)
              && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));
    }

    // JIT alert
    jitAlert.set(isJITing());

    // Record cycle time
    LoggedTracer.record("RobotPeriodic");
  }

  /** Returns whether we should wait to enable because JIT optimizations are in progress. */
  public static boolean isJITing() {
    return Timer.getTimestamp() < 45.0;
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    robotContainer.setCameraThrottle(true);

    robotContainer.stopSubsystems();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // Load PathPlanner paths from storage.
    // This will only load before autonomous starts.
    if (DriverStation.isAutonomous() || Constants.getMode() == Constants.Mode.SIM) {
      robotContainer.loadAutonomousPath();
    }
  }

  /** This function is called once when the robot is enabled in any mode. */
  @Override
  public void disabledExit() {
    robotContainer.setCameraThrottle(false);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

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
