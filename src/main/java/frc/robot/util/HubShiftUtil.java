package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotSystem;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.Color;
import frc.robot.subsystems.lights.Lights.Segment;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class HubShiftUtil {
  // Subsystems
  private static final RobotSystem robotSystem = RobotSystem.getInstance();
  private static final Lights lights = robotSystem.getLights();
  private static Shooter shooter = robotSystem.getShooter();

  private static Optional<Boolean> blueActiveFirst = Optional.empty();
  private static double matchStartTime = -1.0;
  private static double prevMatchStartTime = 0.0;
  private static Shift prevShift = Shift.Transition;
  private static Optional<Boolean> prevBlueActiveFirst = Optional.empty();
  private static LoggedDashboardChooser<Optional<Boolean>> activeFirstOverride =
      new LoggedDashboardChooser<>("HubShift/ActiveFirstOverride");

  static {
    activeFirstOverride.addOption("Blue", Optional.of(true));
    activeFirstOverride.addDefaultOption("None", Optional.empty());
    activeFirstOverride.addOption("Red", Optional.of(false));
  }

  public static LoggedNetworkBoolean override =
      new LoggedNetworkBoolean("SmartDashboard/HubShift/Override", false);

  /**
   * The remaining time left in the mach. Before the match starts, when not connected to FMS, or if
   * not in practice mode this will return -1.
   */
  @Getter private static Supplier<Double> matchTime = () -> -1.0;

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
    if (activeFirstOverride.get().isPresent()) {
      blueActiveFirst = activeFirstOverride.get();
    } else {
      final String message = DriverStation.getGameSpecificMessage();
      if (message.length() > 0) {
        blueActiveFirst =
            switch (message.charAt(0)) {
              case 'R' -> Optional.of(true);
              case 'B' -> Optional.of(false);
              default -> Optional.empty();
            };
      } else {
        blueActiveFirst = Optional.empty();
      }
    }

    // If conditions have changed
    if (matchStartTime != prevMatchStartTime
        || (shiftTime.get() != -1.0 && getActiveShift() != prevShift)
        || (blueActiveFirst.isPresent() ^ prevBlueActiveFirst.isPresent())
        || !blueActiveFirst.equals(prevBlueActiveFirst)) {
      // Print if our hub will be active in shift 1
      // None/grey means no vlaid game data is available
      if (blueActiveFirst.isPresent() && DriverStation.getAlliance().isPresent()) {
        SmartDashboard.putString(
            "HubShift/HubActiveFirst",
            ((DriverStation.getAlliance().get() == Alliance.Blue && blueActiveFirst.get())
                    || (DriverStation.getAlliance().get() == Alliance.Red
                        && !blueActiveFirst.get()))
                ? "#4CAF50"
                : "#F44336");
      } else {
        SmartDashboard.putString("HubShift/HubActiveFirst", "");
      }

      // If values are not seeded, set to default. Otherwise, set based on current shift
      if (matchStartTime == -1.0 || (blueActiveFirst.isEmpty() && !DriverStation.isAutonomous())) {
        currentShift = Shift.Transition;
        currentActiveHub = Optional.empty();
        nextActiveHub = Optional.empty();
        shiftTime = () -> -1.0;
        Logger.recordOutput("HubShift/CurrentShift", "");
      } else {
        currentShift = getActiveShift();
        if (currentShift == Shift.Auton) {
          currentActiveHub = Optional.empty();
          nextActiveHub = Optional.empty();
          shiftTime = () -> Math.max(getMatchTime().get(), 0.0);
          Logger.recordOutput("HubShift/CurrentShift", "AUTON");
        } else {
          switch (currentShift) {
            case Transition:
              currentActiveHub = Optional.empty();
              nextActiveHub =
                  blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
              shiftTime = () -> getMatchTime().get() - Shift.Shift1.startTime;
              Logger.recordOutput("HubShift/CurrentShift", "TRANSITION");
              break;

            case Shift1:
              currentActiveHub =
                  blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
              nextActiveHub =
                  blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
              shiftTime = () -> getMatchTime().get() - Shift.Shift2.startTime;
              Logger.recordOutput("HubShift/CurrentShift", "1");
              break;

            case Shift2:
              currentActiveHub =
                  blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
              nextActiveHub =
                  blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
              shiftTime = () -> getMatchTime().get() - Shift.Shift3.startTime;
              Logger.recordOutput("HubShift/CurrentShift", "2");
              break;

            case Shift3:
              currentActiveHub =
                  blueActiveFirst.get() ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
              nextActiveHub =
                  blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
              shiftTime = () -> getMatchTime().get() - Shift.Shift4.startTime;
              Logger.recordOutput("HubShift/CurrentShift", "3");
              break;

            case Shift4:
              currentActiveHub =
                  blueActiveFirst.get() ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);
              nextActiveHub = Optional.empty();
              shiftTime = () -> getMatchTime().get() - Shift.EndGame.startTime;
              Logger.recordOutput("HubShift/CurrentShift", "4");
              break;

            default:
              currentActiveHub = Optional.empty();
              nextActiveHub = Optional.empty();
              shiftTime = () -> Math.max(getMatchTime().get(), 0.0);
              Logger.recordOutput("HubShift/CurrentShift", "END GAME");
              break;
          }
        }
      }

      // Update previous values
      prevMatchStartTime = matchStartTime;
      prevShift = currentShift;
      prevBlueActiveFirst = blueActiveFirst;
    }

    setLEDState();

    // Log shift time
    Logger.recordOutput("HubShift/ShiftSec", shiftTime.get(), Seconds);
    Logger.recordOutput("HubShift/Override", override);
    SmartDashboard.putNumber("AdjustedTime/MatchTime", Math.max(matchTime.get() + 1.0, 1.0));
    SmartDashboard.putNumber("AdjustedTime/ShiftTime", shiftTime.get() + 1);
  }

  /**
   * Seeds the current remaining match time.
   *
   * <p>If in autonomous or the DS is in teleop or auton mode this will set all fields to default.
   *
   * <p>Call this once at disabled exit.
   */
  public static void seedMatchTime() {
    if (DriverStation.getMatchTime() > 1.0) {
      if (DriverStation.isAutonomous()) {
        matchStartTime = Timer.getFPGATimestamp() - 20.0 + DriverStation.getMatchTime();
        matchTime = () -> 20.0 - (Timer.getFPGATimestamp() - matchStartTime);
      } else {
        matchStartTime = Timer.getFPGATimestamp() - 140.0 + DriverStation.getMatchTime();
        matchTime = () -> 140.0 - (Timer.getFPGATimestamp() - matchStartTime);
      }
    } else {
      matchStartTime = -1.0;
      matchTime = () -> -1.0;
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
  public static boolean shootingToStart(double time) {
    return !isHubActive() && (shiftTime.get() - getShotTime()) < time;
  }

  /**
   * Returns true when the hub is about to go inactive.
   *
   * <p>This will remain true for the specified time before the hub is inactive.
   */
  public static boolean shootingToStop(double time) {
    final double shotTime = getShotTime();
    if (shotTime < 3.0) {
      if (time + shotTime - 3 < 0) {
        return !isHubActive() && shiftTime.get() - 22 - time - shotTime > 0;
      } else {
        return shouldShoot()
            && nextActiveHub.isPresent()
            && (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() != nextActiveHub.get())
            && shiftTime.get() - 3 < time - shotTime;
      }
    } else {
      return shouldShoot()
          && nextActiveHub.isPresent()
          && (DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() != nextActiveHub.get())
          && shiftTime.get() + 3 - shotTime < time;
    }
  }

  /**
   * Returns if the shooter should be allowed to shoot and fuel will score. If the target is not the
   * hub this will always return true.
   */
  @AutoLogOutput(key = "HubShift/ShouldShoot")
  public static boolean shouldShoot() {
    double shotTime = getShotTime();
    double shiftTime = getShiftTime().get();
    if (shotTime < 3.0) {
      return !shooter.isTargetHub()
          || override.get()
          || isHubActive()
          || shiftTime - shotTime < 0
          || (shiftTime - 22.0 - shotTime) > 0.0;
    } else {
      return !shooter.isTargetHub() || override.get() || isHubActive() || shiftTime - shotTime < 0;
    }
  }

  public static double getShotTime() {
    return shooter.getFlightTime() + ShooterConstants.hubFilterTime;
  }

  /**
   * Returns the current active shift.
   *
   * <p>If teleop time hasn't been seeded this will return the transition shift.
   */
  private static Shift getActiveShift() {
    if (DriverStation.isAutonomous()) {
      return Shift.Auton;
    }

    final double time = getMatchTime().get();

    if (time == -1.0 || time > Shift.Shift1.startTime) {
      return Shift.Transition;
    } else if (time > Shift.Shift2.startTime) {
      return Shift.Shift1;
    } else if (time > Shift.Shift3.startTime) {
      return Shift.Shift2;
    } else if (time > Shift.Shift4.startTime) {
      return Shift.Shift3;
    } else if (time > Shift.EndGame.startTime) {
      return Shift.Shift4;
    } else {
      return Shift.EndGame;
    }
  }

  public enum Shift {
    Auton(160),
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

  private static void setLEDState() {
    if (DriverStation.isDisabled()) {
      lights.setSolidColor(Color.Bright.RED, Segment.All);
    } else {
      if (shootingToStart(5) || shootingToStop(5)) {
        lights.setStrobeAnimation(Color.Bright.RED, Segment.All, 400);
      } else if (shootingToStart(10) || shootingToStop(10)) {
        lights.setStrobeAnimation(Color.Bright.RED, Segment.All, 250);
      } else if (shootingToStart(15) || shootingToStop(15)) {
        lights.setStrobeAnimation(Color.Bright.RED, Segment.All, 100);
      } else {
        lights.setSolidColor(Color.Bright.RED, Segment.All);
      }
    }
  }
}
