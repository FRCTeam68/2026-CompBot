package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import java.util.Optional;

public class CanbusReader {
  private final CANBus canbus;
  private final Thread thread;
  private Optional<CANBusStatus> status = Optional.empty();

  public CanbusReader(CANBus canbus) {
    this.canbus = canbus;
    thread =
        new Thread(
            () -> {
              while (true) {
                var statusTemp = Optional.of(this.canbus.getStatus());
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
    thread.setName("CanbusReader");
    thread.start();
  }

  public synchronized Optional<CANBusStatus> getStatus() {
    return status;
  }
}
