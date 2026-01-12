package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionConstants.ObjectObservationType;

public class VisionIOSim implements VisionIO {
  private boolean seeNote = false;

  public VisionIOSim() {
    SmartDashboard.putBoolean("Test/seeNote", seeNote);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    seeNote = SmartDashboard.getBoolean("Test/seeNote", false);
    if (seeNote) {
      inputs.objectObservations =
          new ObjectObservation[] {
            new ObjectObservation(25, 20, 1, 1, ObjectObservationType.NOTE),
            new ObjectObservation(-20, 20, 2, 2, ObjectObservationType.NOTE),
            new ObjectObservation(0, 20, 3, 1, ObjectObservationType.NOTE)
          };
    } else {
      inputs.objectObservations = new ObjectObservation[] {};
    }
  }
}
