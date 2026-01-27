package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShiftUtil {
  private static Optional<Boolean> blueActiveFirst = Optional.empty();
  private static double teleopStartTime = -1.0;
  private static double prevTeleopStartTime = -1.0;
  private static Shift prevShift = Shift.Transition;

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
  @AutoLogOutput(key = "Shift/CurrentShift")
  @Getter
  private static Shift currentShift = Shift.Transition;

  /** Override hub active status to always be active. This only affects {@link #isHubActive()}. */
  @AutoLogOutput(key = "Shift/Override")
  @Setter
  private static boolean override = false;

  /**
   * Update Shift conditions.
   *
   * <p>This should be called periodically.
   */
  public static void update() {
    // FIXME: this can cause a bug when the robot connects from the DS to FMS and will retain the
    // old game message from the DS. This can be fixed by always checking the game message (how long
    // does it take to compare a single character string) or by resetting blueActiveFirst on
    // disabled exit (by then the fms will be reporting the correct data)

    // Seed first active alliance
    // When FMS is not attached this will continually seed
    if (blueActiveFirst.isEmpty() || !DriverStation.isFMSAttached()) {
      blueActiveFirst =
          switch (DriverStation.getGameSpecificMessage()) {
            case "R" -> Optional.of(true);
            case "B" -> Optional.of(false);
            default -> Optional.empty();
          };

      // Print if our hub will be active in shift 1
      // White means data is missing
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
    }

    // If conditions have changed
    if (teleopStartTime != prevTeleopStartTime || getActiveShift() != prevShift) {

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
            break;

          case Shift1:
            currentActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
            nextActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
            shiftTime = () -> getTeleopTime().get() - Shift.Shift2.startTime;
            break;

          case Shift2:
            currentActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
            nextActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
            shiftTime = () -> getTeleopTime().get() - Shift.Shift3.startTime;
            break;

          case Shift3:
            currentActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
            nextActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
            shiftTime = () -> getTeleopTime().get() - Shift.Shift4.startTime;
            break;

          case Shift4:
            currentActiveHub =
                blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
            nextActiveHub = Optional.empty();
            shiftTime = () -> getTeleopTime().get() - Shift.EndGame.startTime;
            break;

          case EndGame:
            currentActiveHub = Optional.empty();
            nextActiveHub = Optional.empty();
            shiftTime = () -> Math.max(getTeleopTime().get(), 0.0);
            break;
        }
      }

      // Update previous values
      prevTeleopStartTime = teleopStartTime;
      prevShift = currentShift;
    }

    // Logging
    Logger.recordOutput("Shift/ShiftSec", shiftTime.get());
  }

  /**
   * Seeds the current remaining match time.
   *
   * <p>If the DS is in teleop or auton mode this will set all fields to default.
   *
   * <p>Call this once at disabled exit.
   */
  public static void seedMatchTime() {
    if (DriverStation.isFMSAttached() || DriverStation.getMatchTime() > 21.0) {
      teleopStartTime = Timer.getFPGATimestamp() - 140.0 + DriverStation.getMatchTime();
    } else {
      teleopStartTime = -1.0;
    }
  }

  /** Returns true if our hub is currently active. */
  @AutoLogOutput(key = "Shift/HubActive")
  public static boolean isHubActive() {
    return currentActiveHub.isEmpty()
        || (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == currentActiveHub.get())
        || override;
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

  /**
   * Returns the current active shift.
   *
   * <p>If teleop time hasn't been seeded this will return the transition shift.
   */
  private static Shift getActiveShift() {
    double teleopTime = getTeleopTime().get();
    Shift shift;

    if (teleopTime == -1.0 || teleopTime > Shift.Shift1.startTime) {
      shift = Shift.Transition;
    } else if (teleopTime > Shift.Shift2.startTime) {
      shift = Shift.Shift1;
    } else if (teleopTime > Shift.Shift3.startTime) {
      shift = Shift.Shift2;
    } else if (teleopTime > Shift.Shift4.startTime) {
      shift = Shift.Shift3;
    } else if (teleopTime > Shift.EndGame.startTime) {
      shift = Shift.Shift4;
    } else {
      shift = Shift.EndGame;
    }

    return shift;
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
