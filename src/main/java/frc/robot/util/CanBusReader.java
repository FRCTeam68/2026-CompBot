package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import java.util.Optional;

public class CanBusReader {
  private final CANBus canBus;
  private final Thread thread;
  private Optional<CANBusStatus> status = Optional.empty();

  public CanBusReader(CANBus canBus) {
    this.canBus = canBus;
    thread =
        new Thread(
            () -> {
              while (true) {
                var statusTemp = Optional.of(this.canBus.getStatus());
                synchronized (this) {
                  status = statusTemp;
                }
                try {
                  Thread.sleep(400); // Match RIO CAN sampling
                } catch (InterruptedException e) {
                  e.printStackTrace();
                }
              }
            });
    thread.setName("CanBusReader");
    thread.start();
  }

  public synchronized Optional<CANBusStatus> getStatus() {
    return status;
  }
}
