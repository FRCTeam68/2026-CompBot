package frc.robot;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.AllianceFlipUtil.SymmetryType;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final SymmetryType symmetryType = SymmetryType.Mirrored;
  public static final AprilTagFieldLayout defaultAprilTagLayout =
      AprilTagLayoutType.OFFICIAL_2026.getLayout();

  public static final double fieldLength = defaultAprilTagLayout.getFieldLength();
  public static final double fieldWidth = defaultAprilTagLayout.getFieldWidth();

  public static final Translation2d blueSpeaker = new Translation2d(1, 1);
  public static final Translation2d redSpeaker = new Translation2d(0, 0);

  @Getter
  public enum AprilTagLayoutType {
    OFFICIAL_2026("2026-official"),
    OFFICIAL_2025("2025-official"),
    OFFICIAL_2024("2024-official");

    AprilTagLayoutType(String name) {
      try {
        layout =
            new AprilTagFieldLayout(
                Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      try {
        layoutString = new ObjectMapper().writeValueAsString(layout);
      } catch (JsonProcessingException e) {
        throw new RuntimeException("Failed to serialize AprilTag layout JSON " + toString());
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }

  public enum FieldType {
    OFFICIAL,
    PRACTICE
  }
}
