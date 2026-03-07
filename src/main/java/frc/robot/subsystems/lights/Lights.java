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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Lights extends SubsystemBase {
  // Default values
  @Getter private static final double onboardLEDBrightness = 0.5;
  private final LoggedTunableNumber defaultAnimationSpeed =
      new LoggedTunableNumber("CANdle/Default Animation Speed", 200);

  // Alerts
  private final Alert disconnectedAlert =
      new Alert("CANdle disconnected.", Alert.AlertType.kWarning);

  // Debouncers
  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  // Sources

  private final LightsIO io;
  protected final LightsIOInputsAutoLogged inputs = new LightsIOInputsAutoLogged();

  public Lights(LightsIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CANdle", inputs);
    disconnectedAlert.set(
        !connectedDebouncer.calculate(inputs.connected) && Constants.getMode() != Mode.SIM);
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
  public void clearAnimation(Segment segment) {
    io.setControl(new EmptyAnimation(segment.animationSlot));

    for (int i = 0; i < segment.overlappingAnimationSlots.length; i++) {
      io.setControl(new EmptyAnimation(segment.overlappingAnimationSlots[i]));
    }
  }

  /**
   * Turn off LEDs of a segment
   *
   * @param segment LED segment to turn off
   */
  public void disableLEDs(Segment segment) {
    setSolidColor(Constants.LEDColor.BLACK, segment);
  }

  /**
   * Set static color for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply color change
   */
  public void setSolidColor(RGBWColor color, Segment segment) {
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
      RGBWColor color, Segment segment, AnimationDirectionValue direction, double... speed) {
    clearAnimation(segment);
    io.setControl(
        new ColorFlowAnimation(segment.startIndex, segment.endIndex)
            .withColor(color)
            .withDirection(direction)
            .withFrameRate(speed.length > 0 ? speed[0] : defaultAnimationSpeed.get())
            .withSlot(segment.animationSlot));
  }

  /**
   * Set fading animation for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   * @param speed How fast should the color travel the strip [0, 1]
   */
  public void setSingleFadeAnimation(RGBWColor color, Segment segment, double... speed) {
    clearAnimation(segment);
    io.setControl(
        new SingleFadeAnimation(segment.startIndex, segment.endIndex)
            .withColor(color)
            .withFrameRate(speed.length > 0 ? speed[0] : defaultAnimationSpeed.get())
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
  public void setBandAnimation(RGBWColor color, Segment segment) {
    setBandAnimation(color, segment, defaultAnimationSpeed.get(), LarsonBounceValue.Front, 4);
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
      RGBWColor color, Segment segment, double speed, LarsonBounceValue bounceMode, int size) {
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
  public void setStrobeAnimation(RGBWColor color, Segment segment, double... speed) {
    clearAnimation(segment);
    io.setControl(
        new StrobeAnimation(segment.startIndex, segment.startIndex)
            .withColor(color)
            .withFrameRate(speed.length > 0 ? speed[0] : defaultAnimationSpeed.get())
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
      Segment segment, AnimationDirectionValue direction, double... speed) {
    clearAnimation(segment);
    io.setControl(
        new RainbowAnimation(segment.startIndex, segment.endIndex)
            .withDirection(direction)
            .withFrameRate(speed.length > 0 ? speed[0] : defaultAnimationSpeed.get())
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

  public static class Segment {
    public int startIndex;
    public int endIndex;
    public int animationSlot;
    public int[] overlappingAnimationSlots = {};

    /**
     * LED segment.
     *
     * @param startIndex Where to start the LED segment
     * @param endSize Where to end the LED segment (inclusive)
     * @param animationSlot The animation slot to use for the animation, range is [0,
     *     getMaxSimultaneousAnimationCount()] exclusive
     */
    public Segment(int startIndex, int endIndex, int animationSlot) {
      this.startIndex = startIndex;
      this.endIndex = endIndex;
      this.animationSlot = animationSlot;
    }

    /**
     * Overlapping animation slots to clear with segment.
     *
     * @param overlappingAnimationSlots Animation slots of segments which contain overlapping LEDs
     *     with the current segment. Clears animations of overlapping to avoid multiple animations
     *     playing simultaneously.
     */
    public Segment withOverlappingAnimationSlots(int... overlappingAnimationSlots) {
      this.overlappingAnimationSlots = overlappingAnimationSlots;
      return this;
    }
  }
}
