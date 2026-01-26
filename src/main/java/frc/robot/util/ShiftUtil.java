package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShiftUtil {
  // These must be positive
  // TODO: should move to shooterconstants. Do we tie this to robot distance.
  private static final LoggedTunableNumber preShiftTime =
      new LoggedTunableNumber("Shift/PreShiftSec", 1.0);
  private static final LoggedTunableNumber postShiftTime =
      new LoggedTunableNumber("Shift/PostShiftSec", 1.0);

  private static Optional<Boolean> blueActiveFirst = Optional.empty();
  private static double teleopStartTime = -1.0;
  private static double prevTeleopStartTime = -1.0;
  private static Shift prevShift = Shift.Transition;
  @Getter private static Supplier<Double> teleopTime = () -> 140.0;
  @Getter private static Supplier<Double> shiftTime = () -> 30.0;
  @Getter private static Optional<Alliance> currentActive = Optional.empty();
  @Getter private static Optional<Alliance> nextActive = Optional.empty();

  @AutoLogOutput(key = "Shift/CurrentShift")
  @Getter
  private static Shift currentShift = Shift.Transition;

  @AutoLogOutput(key = "Shift/Override")
  @Setter
  private static boolean override = false;

  /**
   * Update Shift conditions.
   *
   * <p>This should be called periodically during teleop mode.
   */
  public static void update() {
    // TODO: This way we don't need to read the string once it is declared. It the perfromance hit
    // even that bad?
    // Seed first active alliance
    if (blueActiveFirst.isEmpty() || !DriverStation.isFMSAttached()) {
      blueActiveFirst =
          switch (DriverStation.getGameSpecificMessage()) {
            case "R" -> Optional.of(true);
            case "B" -> Optional.of(false);
            default -> Optional.empty();
          };
    }

    // If conditions have changed
    if (teleopStartTime != prevTeleopStartTime || getActiveShift() != prevShift) {

      // Configure teleop time
      teleopTime =
          () ->
              (teleopStartTime == -1)
                  ? 140.0
                  : 140.0 - (Timer.getFPGATimestamp() - teleopStartTime);

      // If values are not seeded, set to default. Otherwise, set based on current shift
      if (teleopStartTime == -1.0 || blueActiveFirst.isEmpty()) {
        currentShift = Shift.Transition;
        currentActive = Optional.empty();
        nextActive = Optional.empty();
        shiftTime = () -> 30.0;
      } else {
        currentShift = getActiveShift();
        switch (currentShift) {
          case Transition:
            currentActive = Optional.empty();
            nextActive =
                blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
            shiftTime = () -> getTeleopTime().get() - Shift.Shift1.startTime;
            break;

          case Shift1:
            currentActive =
                blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
            nextActive =
                blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
            shiftTime = () -> getTeleopTime().get() - Shift.Shift2.startTime;
            break;

          case Shift2:
            currentActive =
                blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
            nextActive =
                blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
            shiftTime = () -> getTeleopTime().get() - Shift.Shift3.startTime;
            break;

          case Shift3:
            currentActive =
                blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
            nextActive =
                blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
            shiftTime = () -> getTeleopTime().get() - Shift.Shift4.startTime;
            break;

          case Shift4:
            currentActive =
                blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
            nextActive = Optional.empty();
            shiftTime = () -> getTeleopTime().get() - Shift.EndGame.startTime;
            break;

          case EndGame:
            currentActive = Optional.empty();
            nextActive = Optional.empty();
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
   * Seeds the current remaining teleop time.
   *
   * <p>This is no-op when the DS is in teleop mode.
   *
   * <p>Call this once at teleop init.
   */
  public static void seedTeleopTime() {
    if (DriverStation.isFMSAttached() || DriverStation.getMatchTime() > 5.0) {
      teleopStartTime = Timer.getFPGATimestamp() - 140.0 + DriverStation.getMatchTime();
    } else {
      teleopStartTime = -1.0;
    }
  }

  /**
   * Returns true when the robot will be able to shoot and have the fuel score.
   *
   * <p>This is preferred for shooting over isHubActive since it accounts for flight time and the 3
   * seconds fuel can score after the hub deactivates.
   */
  @AutoLogOutput(key = "Shift/CanShoot")
  public static boolean canShoot() {
    return ishubActive()
        || shiftTime.get() + postShiftTime.get() > 25.0
        || shiftTime.get() - preShiftTime.get() < 0.0
        || override;
  }

  /** Returns true if our hub is currently active. */
  @AutoLogOutput(key = "Shift/HubActive")
  public static boolean ishubActive() {
    return currentActive.isEmpty()
        || (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == currentActive.get())
        || override;
  }

  /**
   * Returns true when the hub is about to go active.
   *
   * <p>This will remain true for the specified time before the hub is active.
   */
  public static boolean hubToActiveWarning(double warningTime) {
    return !ishubActive() && shiftTime.get() < warningTime;
  }

  /**
   * Returns true when the hub is about to go inactive.
   *
   * <p>This will remain true for the specified time before the hub is inactive.
   */
  public static boolean hubToInactiveWarning(double warningTime) {
    return ishubActive()
        && nextActive.isPresent()
        && (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() != nextActive.get())
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
