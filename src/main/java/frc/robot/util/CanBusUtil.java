package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CanBusUtil {
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final Timer canInitialErrorTimer = new Timer();
  private static final Timer canivoreErrorTimer = new Timer();
  private static final Timer rioErrorTimer = new Timer();
  private static final Alert rioErrorAlert =
      new Alert("Rio CAN errors detected, robot may not be controllable.", AlertType.kError);
  private static final Alert canivoreErrorAlert =
      new Alert("CANivore CAN errors detected, robot may not be controllable.", AlertType.kError);

  @Getter private static final CANBus rioBus = new CANBus("rio");
  @Getter private static final CANBus canivoreBus = new CANBus("*");
  private static final CanBusReader rioReader = new CanBusReader(rioBus);
  private static final CanBusReader canivoreReader = new CanBusReader(canivoreBus);

  public static void logStatus() {
    if (Constants.getMode() == Mode.REAL) {
      // rioBus status
      final var rioStatus = rioReader.getStatus();
      if (rioStatus.isPresent()) {
        Logger.recordOutput("CANBusStatus/Rio/Status", rioStatus.get().Status.getName());
        Logger.recordOutput("CANBusStatus/Rio/Utilization", rioStatus.get().BusUtilization);
        Logger.recordOutput("CANBusStatus/Rio/OffCount", rioStatus.get().BusOffCount);
        Logger.recordOutput("CANBusStatus/Rio/TxFullCount", rioStatus.get().TxFullCount);
        Logger.recordOutput("CANBusStatus/Rio/ReceiveErrorCount", rioStatus.get().REC);
        Logger.recordOutput("CANBusStatus/Rio/TransmitErrorCount", rioStatus.get().TEC);
        if (!rioStatus.get().Status.isOK() || rioStatus.get().TEC > 0 || rioStatus.get().REC > 0) {
          rioErrorTimer.restart();
        }
      }
      rioErrorAlert.set(
          !rioErrorTimer.hasElapsed(canErrorTimeThreshold)
              && canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

      // CANivoreBus status
      final var canivoreStatus = canivoreReader.getStatus();
      if (canivoreStatus.isPresent()) {
        Logger.recordOutput("CANBusStatus/CANivore/Status", canivoreStatus.get().Status.getName());
        Logger.recordOutput(
            "CANBusStatus/CANivore/Utilization", canivoreStatus.get().BusUtilization);
        Logger.recordOutput("CANBusStatus/CANivore/OffCount", canivoreStatus.get().BusOffCount);
        Logger.recordOutput("CANBusStatus/CANivore/TxFullCount", canivoreStatus.get().TxFullCount);
        Logger.recordOutput("CANBusStatus/CANivore/ReceiveErrorCount", canivoreStatus.get().REC);
        Logger.recordOutput("CANBusStatus/CANivore/TransmitErrorCount", canivoreStatus.get().TEC);
        if (!canivoreStatus.get().Status.isOK()
            || canivoreStatus.get().TEC > 0
            || canivoreStatus.get().REC > 0) {
          canivoreErrorTimer.restart();
        }
      }
      canivoreErrorAlert.set(
          !canivoreErrorTimer.hasElapsed(canErrorTimeThreshold)
              && canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));
    }
  }
}
