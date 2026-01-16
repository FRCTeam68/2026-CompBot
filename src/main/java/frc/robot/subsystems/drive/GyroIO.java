package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public double yawVelocityRadPerSec = 0.0;
    public Rotation2d pitchPosition = Rotation2d.kZero;
    public double pitchVelocityRadPerSec = 0.0;
    public Rotation2d rollPosition = Rotation2d.kZero;
    public double rollVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
