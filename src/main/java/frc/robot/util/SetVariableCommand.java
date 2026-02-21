package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class SetVariableCommand {
  /**
   * Creates a command to update the value of a variable.
   *
   * <p>The command will run while disabled.
   *
   * <p><b>Example call:</b>
   *
   * <pre>SetVariableWithCommand.apply(v -> manualShootToggle = v, () -> true);
   * </pre>
   */
  public static Command apply(Consumer<Boolean> booleanVariable, Supplier<Boolean> value) {
    return Commands.runOnce(() -> booleanVariable.accept(value.get()))
        .ignoringDisable(true)
        .withName("SetVariable");
  }
}
