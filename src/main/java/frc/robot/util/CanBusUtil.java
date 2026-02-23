package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class CanBusUtil {
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final double rioErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final double canivoreErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final Timer canInitialErrorTimer = new Timer();
  private static final Timer canErrorTimer = new Timer();
  private static final Timer canivoreErrorTimer = new Timer();
  private static final Timer rioErrorTimer = new Timer();
  private static final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  private static final Alert rioErrorAlert =
      new Alert("Rio CAN errors detected, robot may not be controllable.", AlertType.kError);
  private static final Alert canivoreErrorAlert =
      new Alert("CANivore CAN errors detected, robot may not be controllable.", AlertType.kError);

  private static CANBus rioBus = null;
  private static CANBus canivoreBus = null;
  private static CanBusReader rioReader;
  private static CanBusReader canivoreReader;

  public static void logStatus() {
    if (Constants.getMode() != Mode.SIM) {
      // Check CAN status
      var canStatus = RobotController.getCANStatus();
      if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
        canErrorTimer.restart();
      }
      canErrorAlert.set(
          !canErrorTimer.hasElapsed(canErrorTimeThreshold)
              && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

      // Log rioBus status
      if (Constants.getMode() == Constants.Mode.REAL) {
        if (rioBus != null) {
          var rioStatus = rioReader.getStatus();
          if (rioStatus.isPresent()) {
            Logger.recordOutput("RioStatus/Status", rioStatus.get().Status.getName());
            Logger.recordOutput("RioStatus/Utilization", rioStatus.get().BusUtilization);
            Logger.recordOutput("RioStatus/OffCount", rioStatus.get().BusOffCount);
            Logger.recordOutput("RioStatus/TxFullCount", rioStatus.get().TxFullCount);
            Logger.recordOutput("RioStatus/ReceiveErrorCount", rioStatus.get().REC);
            Logger.recordOutput("RioStatus/TransmitErrorCount", rioStatus.get().TEC);
            if (!rioStatus.get().Status.isOK()
                || canStatus.transmitErrorCount > 0
                || canStatus.receiveErrorCount > 0) {
              rioErrorTimer.restart();
            }
          }
          rioErrorAlert.set(
              !rioErrorTimer.hasElapsed(rioErrorTimeThreshold)
                  && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));
        }

        if (canivoreBus != null) {
          var canivoreStatus = canivoreReader.getStatus();
          if (canivoreStatus.isPresent()) {
            Logger.recordOutput("CANivoreStatus/Status", canivoreStatus.get().Status.getName());
            Logger.recordOutput("CANivoreStatus/Utilization", canivoreStatus.get().BusUtilization);
            Logger.recordOutput("CANivoreStatus/OffCount", canivoreStatus.get().BusOffCount);
            Logger.recordOutput("CANivoreStatus/TxFullCount", canivoreStatus.get().TxFullCount);
            Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", canivoreStatus.get().REC);
            Logger.recordOutput("CANivoreStatus/TransmitErrorCount", canivoreStatus.get().TEC);
            if (!canivoreStatus.get().Status.isOK()
                || canStatus.transmitErrorCount > 0
                || canStatus.receiveErrorCount > 0) {
              canivoreErrorTimer.restart();
            }
          }
          canivoreErrorAlert.set(
              !canivoreErrorTimer.hasElapsed(canivoreErrorTimeThreshold)
                  && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));
        }
      }
    }
  }

  public static CANBus getRioBus() {
    if (rioBus == null) {
      rioBus = new CANBus("rio");
      rioReader = new CanBusReader(rioBus);
    }
    return rioBus;
  }

  public static CANBus getCanivoreBus() {
    if (canivoreBus == null) {
      canivoreBus = new CANBus("*");
      canivoreReader = new CanBusReader(canivoreBus);
    }
    return canivoreBus;
  }
}
