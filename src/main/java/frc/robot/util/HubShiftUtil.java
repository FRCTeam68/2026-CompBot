package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HubShiftUtil {
  private static Optional<Boolean> blueActiveFirst = Optional.empty();
  private static double teleopStartTime = -1.0;
  private static double prevTeleopStartTime = 0.0;
  private static Shift prevShift = Shift.Transition;
  private static Optional<Boolean> prevBlueActiveFirst = Optional.empty();

  /**
   * The remaining time in teleop mode. Before teleop starts, when not connected to FMS, or if not
   * in practice mode this will return -1.
   */
  @Getter private static Supplier<Double> teleopTime = () -> -1.0;

  /**
   * The remaining time in the current shift. Before teleop starts, when not connected to FMS, or if
   * not in practice mode this will return -1.
   */
  @Getter private static Supplier<Double> shiftTime = () -> -1.0;

  /** The current active hub. If both hubs are active an empty optional will be returned instead. */
  @Getter private static Optional<Alliance> currentActiveHub = Optional.empty();

  /**
   * The active hub for the next shift. If both hubs will be active an empty optional will be
   * returned instead.
   */
  @Getter private static Optional<Alliance> nextActiveHub = Optional.empty();

  /**
   * The current shift. Before teleop starts, when not connected to FMS, or if not in practice mode
   * this will return the Transition shift.
   */
  @Getter private static Shift currentShift = Shift.Transition;

  /**
   * Update Shift conditions.
   *
   * <p>This should be called periodically.
   */
  public static void update() {
    // Read game specific data
    // DriverStation.getGameSpecificMessage() is not cleared when no data is entered
    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      blueActiveFirst =
          switch (message.charAt(0)) {
            case 'R' -> Optional.of(true);
            case 'B' -> Optional.of(false);
            default -> Optional.empty();
          };
    }

    // If conditions have changed
    if (teleopStartTime != prevTeleopStartTime
        || (shiftTime.get() != -1.0 && getActiveShift() != prevShift)
        || (blueActiveFirst.isPresent() ^ prevBlueActiveFirst.isPresent())
        || !blueActiveFirst.equals(prevBlueActiveFirst)) {
      // Print if our hub will be active in shift 1
      // None/grey means no vlaid game data is available
      if (blueActiveFirst.isPresent() && DriverStation.getAlliance().isPresent()) {
        SmartDashboard.putString(
            "HubActiveFirst",
            ((DriverStation.getAlliance().get() == Alliance.Blue && blueActiveFirst.get())
                    || (DriverStation.getAlliance().get() == Alliance.Red
                        && !blueActiveFirst.get()))
                ? "#4CAF50"
                : "#F44336");
      } else {
        SmartDashboard.putString("HubActiveFirst", "");
      }

      // Configure teleop time
      teleopTime =
          () ->
              (teleopStartTime == -1) ? -1.0 : 140.0 - (Timer.getFPGATimestamp() - teleopStartTime);

      // If values are not seeded, set to default. Otherwise, set based on current shift
      if (teleopStartTime == -1.0 || blueActiveFirst.isEmpty()) {
        currentShift = Shift.Transition;
        currentActiveHub = Optional.empty();
        nextActiveHub = Optional.empty();
        shiftTime = () -> -1.0;
      } else {
        currentShift = getActiveShift();
        switch (currentShift) {
          case Transition:
            currentActiveHub = Optional.empty();
            nextActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
            shiftTime = () -> getTeleopTime().get() - Shift.Shift1.startTime;
            Logger.recordOutput("HubShift/CurrentShift", "TRANSITION");
            break;

          case Shift1:
            currentActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
            nextActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
            shiftTime = () -> getTeleopTime().get() - Shift.Shift2.startTime;
            Logger.recordOutput("HubShift/CurrentShift", "1");
            break;

          case Shift2:
            currentActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
            nextActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
            shiftTime = () -> getTeleopTime().get() - Shift.Shift3.startTime;
            Logger.recordOutput("HubShift/CurrentShift", "2");
            break;

          case Shift3:
            currentActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
            nextActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
            shiftTime = () -> getTeleopTime().get() - Shift.Shift4.startTime;
            Logger.recordOutput("HubShift/CurrentShift", "3");
            break;

          case Shift4:
            currentActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
            nextActiveHub = Optional.empty();
            shiftTime = () -> getTeleopTime().get() - Shift.EndGame.startTime;
            Logger.recordOutput("HubShift/CurrentShift", "4");
            break;

          case EndGame:
            currentActiveHub = Optional.empty();
            nextActiveHub = Optional.empty();
            shiftTime = () -> Math.max(getTeleopTime().get(), 0.0);
            Logger.recordOutput("HubShift/CurrentShift", "END GAME");
            break;
        }
      }

      // Update previous values
      prevTeleopStartTime = teleopStartTime;
      prevShift = currentShift;
      prevBlueActiveFirst = blueActiveFirst;
    }

    // Log shift time
    Logger.recordOutput("HubShift/ShiftSec", shiftTime.get(), Seconds);
  }

  /**
   * Seeds the current remaining match time.
   *
   * <p>If in autonomous or the DS is in teleop or auton mode this will set all fields to default.
   *
   * <p>Call this once at disabled exit.
   */
  public static void seedMatchTime() {
    if (!DriverStation.isAutonomous() && DriverStation.getMatchTime() > 5.0) {
      teleopStartTime = Timer.getFPGATimestamp() - 140.0 + DriverStation.getMatchTime();
    } else {
      teleopStartTime = -1.0;
    }
  }

  /** Returns true if our hub is currently active. */
  @AutoLogOutput(key = "HubShift/HubActive")
  public static boolean isHubActive() {
    return currentActiveHub.isEmpty()
        || (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == currentActiveHub.get());
  }

  /**
   * Returns true when the hub is about to go active.
   *
   * <p>This will remain true for the specified time before the hub is active.
   */
  public static boolean hubToActiveWarning(double warningTime) {
    return !isHubActive() && shiftTime.get() < warningTime;
  }

  /**
   * Returns true when the hub is about to go inactive.
   *
   * <p>This will remain true for the specified time before the hub is inactive.
   */
  public static boolean hubToInactiveWarning(double warningTime) {
    return isHubActive()
        && nextActiveHub.isPresent()
        && (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() != nextActiveHub.get())
        && shiftTime.get() < warningTime;
  }

  /** Returns if the shooter should be allowed to shoot and fuel will score. */
  public static boolean shouldShoot() {
    // TODO: make this use the dynamic flight time
    return isHubActive() || hubToActiveWarning(3);
  }

  /**
   * Returns the current active shift.
   *
   * <p>If teleop time hasn't been seeded this will return the transition shift.
   */
  private static Shift getActiveShift() {
    double teleopTime = getTeleopTime().get();

    if (teleopTime == -1.0 || teleopTime > Shift.Shift1.startTime) {
      return Shift.Transition;
    } else if (teleopTime > Shift.Shift2.startTime) {
      return Shift.Shift1;
    } else if (teleopTime > Shift.Shift3.startTime) {
      return Shift.Shift2;
    } else if (teleopTime > Shift.Shift4.startTime) {
      return Shift.Shift3;
    } else if (teleopTime > Shift.EndGame.startTime) {
      return Shift.Shift4;
    } else {
      return Shift.EndGame;
    }
  }

  public enum Shift {
    Transition(140),
    Shift1(130),
    Shift2(105),
    Shift3(80),
    Shift4(55),
    EndGame(30);

    public int startTime;

    private Shift(int startTime) {
      this.startTime = startTime;
    }
  }
}
