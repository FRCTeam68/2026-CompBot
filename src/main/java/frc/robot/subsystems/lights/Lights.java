package frc.robot.subsystems.lights;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.Logger;

public class Lights extends SubsystemBase {
  // Onboard LEDs
  // 0 - Auton chassis in correct starting position
  // 1 - Turret position ambiguous
  // 2 -
  // 3 - Hopper Sensor
  // 4 - Auton subsystems in starting positions
  // 5 -
  // 6 - LL4
  // 7 - LL3G

  // Default values
  private static final double onboardLEDBrightness = 0.5;
  private final double defaultAnimationSpeed = 200;

  public static class Segment {
    public static final LEDSegment All = new LEDSegment(8, 44, 1);
    public static final LEDSegment Back = new LEDSegment(8, 15, 1);
    public static final LEDSegment Side = new LEDSegment(16, 44, 1);
  }

  public static class Color {
    // team colors
    public static final RGBWColor ORANGE = new RGBWColor(255, 142, 36);
    public static final RGBWColor BLUE = new RGBWColor(0, 0, 255);

    // indicator colors
    public static final RGBWColor BLACK = new RGBWColor(0, 0, 0);
    public static final RGBWColor WHITE = new RGBWColor(255, 230, 220);
    public static final RGBWColor GREEN = new RGBWColor(56, 209, 0);
    public static final RGBWColor RED = new RGBWColor(255, 0, 0);
  }

  // Alerts
  private final Alert disconnectedAlert =
      new Alert("CANdle disconnected.", Alert.AlertType.kWarning);

  // Debouncers
  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private final LightsIO io;
  protected final LightsIOInputsAutoLogged inputs = new LightsIOInputsAutoLogged();

  public Lights(LightsIO io) {
    this.io = io;
  }

  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("CANdle", inputs);

    // Update alerts
    disconnectedAlert.set(
        !connectedDebouncer.calculate(inputs.connected) && Constants.getMode() != Mode.SIM);

    // Onboard LEDs
    if ((DriverStation.isFMSAttached() && DriverStation.isDisabled()) || Constants.tuningMode) {}

    // LED strip
    if (DriverStation.isDisabled()) {
      setSolidColor(Color.ORANGE, Segment.All);
    } else {
      final double animationSpeed;
      if (HubShiftUtil.shootingToStart(3) || HubShiftUtil.shootingToStop(3)) {}
    }
  }

  /**
   * Set brightness for all LEDs
   *
   * @param percent Value from [0, 1] that will scale the LED brightness
   */
  public void setBrightness(double percent) {
    io.setBrightness(percent);
  }

  /**
   * Clear animation of a segment and all overlapping animation slots
   *
   * @param segment LED segment to clear animation
   */
  public void clearAnimation(LEDSegment segment) {
    io.setControl(new EmptyAnimation(segment.animationSlot));
  }

  /**
   * Turn off LEDs of a segment
   *
   * @param segment LED segment to turn off
   */
  public void disableLEDs(LEDSegment segment) {
    setSolidColor(Color.BLACK, segment);
  }

  /**
   * Set static color for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply color change
   */
  public void setSolidColor(RGBWColor color, LEDSegment segment) {
    clearAnimation(segment);
    io.setControl(new SolidColor(segment.startIndex, segment.endIndex).withColor(color));
  }

  /**
   * Set flowing animation for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   * @param direction What direction should the color move in
   * @param speed How fast should the color travel the strip [0, 1]
   */
  public void setFlowAnimation(
      RGBWColor color, LEDSegment segment, AnimationDirectionValue direction, double... speed) {
    clearAnimation(segment);
    io.setControl(
        new ColorFlowAnimation(segment.startIndex, segment.endIndex)
            .withColor(color)
            .withDirection(direction)
            .withFrameRate(speed.length > 0 ? speed[0] : defaultAnimationSpeed)
            .withSlot(segment.animationSlot));
  }

  /**
   * Set fading animation for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   * @param speed How fast should the color travel the strip [0, 1]
   */
  public void setSingleFadeAnimation(RGBWColor color, LEDSegment segment, double... speed) {
    clearAnimation(segment);
    io.setControl(
        new SingleFadeAnimation(segment.startIndex, segment.endIndex)
            .withColor(color)
            .withFrameRate(speed.length > 0 ? speed[0] : defaultAnimationSpeed)
            .withSlot(segment.animationSlot));
  }

  /**
   * Set banding animation for LED segment
   *
   * <p>using default values
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   */
  public void setBandAnimation(RGBWColor color, LEDSegment segment) {
    setBandAnimation(color, segment, defaultAnimationSpeed, LarsonBounceValue.Front, 4);
  }

  /**
   * Set banding animation for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   * @param speed How fast should the color travel the strip [0, 1]
   * @param mode How the pocket of LEDs will behave once it reaches the end of the strip
   * @param size How large the pocket of LEDs are [0, 15]
   */
  public void setBandAnimation(
      RGBWColor color, LEDSegment segment, double speed, LarsonBounceValue bounceMode, int size) {
    clearAnimation(segment);
    io.setControl(
        new LarsonAnimation(segment.startIndex, segment.endIndex)
            .withBounceMode(bounceMode)
            .withColor(color)
            .withFrameRate(speed)
            .withSize(size)
            .withSlot(segment.animationSlot));
  }

  /**
   * Set strobing animation for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   * @param speed How fast should the color travel the strip [0, 1]
   */
  public void setStrobeAnimation(RGBWColor color, LEDSegment segment, double... speed) {
    clearAnimation(segment);
    io.setControl(
        new StrobeAnimation(segment.startIndex, segment.startIndex)
            .withColor(color)
            .withFrameRate(speed.length > 0 ? speed[0] : defaultAnimationSpeed)
            .withSlot(segment.animationSlot));
  }

  /**
   * Set rainbow animation for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   * @param reverseDirection True to reverse the animation direction, so instead of going "toward"
   *     the CANdle, it will go "away" from the CANdle.
   * @param speed How fast should the color travel the strip [0, 1]
   */
  public void setRainbowAnimation(
      LEDSegment segment, AnimationDirectionValue direction, double... speed) {
    clearAnimation(segment);
    io.setControl(
        new RainbowAnimation(segment.startIndex, segment.endIndex)
            .withDirection(direction)
            .withFrameRate(speed.length > 0 ? speed[0] : defaultAnimationSpeed)
            .withSlot(segment.animationSlot));
  }

  /**
   * Control LEDs with abstract control request object.
   *
   * @param request Abstract Control Request class that other control requests extend for use
   */
  public void setControlRequest(ControlRequest request) {
    io.setControl(request);
  }

  public static class LEDSegment {
    public int startIndex;
    public int endIndex;
    public int animationSlot;

    /**
     * LED segment.
     *
     * @param startIndex Where to start the LED segment
     * @param endSize Where to end the LED segment (inclusive)
     * @param animationSlot The animation slot to use for the animation, range is [0,
     *     getMaxSimultaneousAnimationCount()] exclusive
     */
    public LEDSegment(int startIndex, int endIndex, int animationSlot) {
      this.startIndex = startIndex;
      this.endIndex = endIndex;
      this.animationSlot = animationSlot;
    }
  }
}
