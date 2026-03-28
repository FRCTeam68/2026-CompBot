package frc.robot.util;

import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.units.measure.MutPower;
import edu.wpi.first.units.measure.Power;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VirtualPD {
  private static ArrayList<Supplier<Power>> motors = new ArrayList<>();
  private static ArrayList<String> groups = new ArrayList<>();
  private static final double deltatime = 0.02;
  // Accumulated total energy per group across calls to logTotalCurrent()
  private static final HashMap<String, MutPower> groupTotalsAccum = new HashMap<>();
  // Accumulated total energy across all groups (persistent across calls)
  private static final MutPower accumulatedTotal = Watts.of(0).mutableCopy();

  public static void registerMotor(Supplier<Power> powerSupplier, String group) {
    motors.add(powerSupplier);
    groups.add(group);
  }

  public static void logTotalCurrent() {
    MutPower total = Watts.of(0).mutableCopy();
    // local instantaneous group totals for this call (not accumulated)
    HashMap<String, Power> groupTotals = new HashMap<>();

    for (int i = 0; i < motors.size(); i++) {
      Power power = motors.get(i).get().times(deltatime);
      total.mut_plus(power);
      // Add to global accumulated total
      accumulatedTotal.mut_plus(power);
      String group = groups.get(i);
      if (groupTotals.containsKey(group)) {
        groupTotals.put(group, groupTotals.get(group).plus(power));
      } else {
        groupTotals.put(group, power);
      }

      // Update accumulated totals (mutating to avoid allocations where possible)
      if (groupTotalsAccum.containsKey(group)) {
        groupTotalsAccum.get(group).mut_plus(power);
      } else {
        groupTotalsAccum.put(group, power.mutableCopy());
      }
    }

    Logger.recordOutput("VirtualPD/Total", total);
    for (String group : groupTotals.keySet()) {
      Logger.recordOutput("VirtualPD/" + group, groupTotals.get(group));
    }

    // Also log the accumulated total per-group across calls
    for (Map.Entry<String, MutPower> e : groupTotalsAccum.entrySet()) {
      Logger.recordOutput("VirtualPD/Accumulated/" + e.getKey(), e.getValue());
    }
    // Log the overall accumulated total across all groups
    Logger.recordOutput("VirtualPD/Accumulated/Total", accumulatedTotal);
  }
}
