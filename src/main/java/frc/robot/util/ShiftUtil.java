package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class ShiftUtil {
  // These must be positive
  // TODO: these should move to shooterconstants
  private static final LoggedTunableNumber preShiftTime =
      new LoggedTunableNumber("Shift/PreShiftSec", 1.0);
  private static final LoggedTunableNumber postShiftTime =
      new LoggedTunableNumber("Shift/PostShiftSec", 1.0);

  private static Optional<Boolean> blueActiveFirst = Optional.empty();
  private static double teleopStartTime = -1.0;
  @Getter private static double teleopTime = 140.0;
  private static Shift currentShift = Shift.Transition;
  @Getter private static Alliance currentActive = Alliance.Both;
  @Getter private static Alliance nextActive = Alliance.Both;
  @Getter private static double shiftTime = 30.0;
  @Getter private static boolean canShoot = true;
  @Setter private static boolean override = false;

  public static void calculate() {
    seedAlliance();

    teleopTime =
        (teleopStartTime == -1) ? 140.0 : 140.0 - (Timer.getFPGATimestamp() - teleopStartTime);

    if (teleopStartTime == -1.0 || blueActiveFirst.isEmpty()) {
      currentShift = Shift.Transition;
      currentActive = Alliance.Both;
      nextActive = Alliance.Both;
      shiftTime = 30.0;
    } else {
      currentShift = getCurrentShift();
      switch (currentShift) {
        case Transition:
          currentActive = Alliance.Both;
          nextActive = blueActiveFirst.get() ? Alliance.Blue : Alliance.Red;
          shiftTime = getTeleopTime() - Shift.Shift1.startTime;
          break;

        case Shift1:
          currentActive = blueActiveFirst.get() ? Alliance.Blue : Alliance.Red;
          nextActive = blueActiveFirst.get() ? Alliance.Red : Alliance.Blue;
          shiftTime = getTeleopTime() - Shift.Shift2.startTime;
          break;

        case Shift2:
          currentActive = blueActiveFirst.get() ? Alliance.Red : Alliance.Blue;
          nextActive = blueActiveFirst.get() ? Alliance.Blue : Alliance.Red;
          shiftTime = getTeleopTime() - Shift.Shift3.startTime;
          break;

        case Shift3:
          currentActive = blueActiveFirst.get() ? Alliance.Blue : Alliance.Red;
          nextActive = blueActiveFirst.get() ? Alliance.Red : Alliance.Blue;
          shiftTime = getTeleopTime() - Shift.Shift4.startTime;
          break;

        case Shift4:
          currentActive = blueActiveFirst.get() ? Alliance.Red : Alliance.Blue;
          nextActive = Alliance.Both;
          shiftTime = getTeleopTime() - Shift.EndGame.startTime;
          break;

        case EndGame:
          currentActive = Alliance.Both;
          nextActive = Alliance.Both;
          shiftTime = Math.max(getTeleopTime(), 0.0);
          break;
      }
      ;
    }

    if (currentActive == Alliance.Both
        || DriverStation.getAlliance().get() == currentActive.toWPIAlliance().get()
        || override) {
      canShoot = true;
    } else if (shiftTime + postShiftTime.get() > 25.0 || shiftTime - preShiftTime.get() < 0.0) {
      canShoot = true;
    } else {
      canShoot = false;
    }

    Logger.recordOutput("Shift/CurrentActive", currentActive);
    Logger.recordOutput("Shift/ShiftTimeSec", shiftTime);
    Logger.recordOutput("Shift/CanShoot", canShoot);
  }

  /** Call this once at teleop init. */
  public static void seedMatchTime() {
    // TODO: I hate that I have to do it this way, but there is no way to know if we are in practice
    // mode or not
    if (DriverStation.isFMSAttached() || DriverStation.getMatchTime() > 5.0) {
      teleopStartTime = Timer.getFPGATimestamp() - 140.0 + DriverStation.getMatchTime();
    } else {
      teleopStartTime = -1.0;
    }
  }

  private static void seedAlliance() {
    // TODO: This way we don't need to read the string once it is declared. It the perfromance hit
    // even that bad?
    if (blueActiveFirst.isEmpty() || !DriverStation.isFMSAttached()) {
      switch (DriverStation.getGameSpecificMessage()) {
        case "R":
          blueActiveFirst = Optional.of(true);
          break;

        case "B":
          blueActiveFirst = Optional.of(false);
          break;

        default:
          blueActiveFirst = Optional.empty();
          break;
      }
    }
  }

  private static Shift getCurrentShift() {
    double teleopTime = getTeleopTime();
    Shift shift;

    if (teleopTime > Shift.Shift1.startTime) {
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

  public enum Alliance {
    Red,
    Blue,
    Both;

    public Optional<edu.wpi.first.wpilibj.DriverStation.Alliance> toWPIAlliance() {
      return switch (this) {
        case Red -> Optional.of(edu.wpi.first.wpilibj.DriverStation.Alliance.Red);
        case Blue -> Optional.of(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
        case Both -> Optional.empty();
      };
    }
  }
}
