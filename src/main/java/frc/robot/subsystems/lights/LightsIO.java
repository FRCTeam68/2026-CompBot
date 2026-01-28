package frc.robot.subsystems.lights;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface LightsIO {
  @AutoLog
  static class LightsIOInputs {
    public boolean connected = false;
    public double outputCurrent = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(LightsIOInputs inputs) {}

  /**
   * Set brightness for all LEDs
   *
   * @param percent Value from [0, 1] that will scale the LED brightness
   */
  default void setBrightness(double percent) {}

  /**
   * Control LEDs with generic control request object
   *
   * @param request Abstract Control Request class that other control requests extend for use
   */
  default void setControl(ControlRequest request) {}
}
