package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;

// From FRC 2910
// this is used to store information on the field mesurements
public class FieldConstants {

  public static final double FIELD_WIDTH = Units.feetToMeters(54.27083);
  public static final double FIELD_LENGTH = Units.feetToMeters(26.9375);
  public static final AprilTagFieldLayout FIELD_LAYOUT;

  // This is in a static block so the code will run when first referenced
  static {
    // this block loads data from a resource which may throw a I/O Exception if it isnt loaded
    // If the resource isnt loaded it will try again, if unchecked the Field layout would be empty (Not good)
    try {
      FIELD_LAYOUT =
        AprilTagFieldLayout.loadFromResource(
          AprilTagFields.k2024Crescendo.m_resourceFile
        );
      FIELD_LAYOUT.setOrigin(
        AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide
      );
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static boolean IsBlueAllience() {
    return (
      DriverStation.getAlliance().isEmpty() ||
      DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
    );
  }

  // Speaker Mesurements
  public static final double SPEAKER_HEIGHT_METERS = Units.inchesToMeters(78);
  public static final double SPEAKER_OPENING_WIDTH_METERS = Units.inchesToMeters(
    41.375
  );
  public static final double SPEAKER_OPENING_EXTENSION_LENGTH_METERS = Units.inchesToMeters(
    18
  );
  public static final double SPEAKER_OPENING_ANGLE_RADIANS = Units.degreesToRadians(
    14
  );
  public static final double LENGTH_BETWEEN_SPEAKER_TAGS_METERS = Units.inchesToMeters(
    17
  );

  public static final double LENGTH_BETWEEN_SOURCE_TAGS_METERS = Units.inchesToMeters(
    38.75
  );
  public static final double LOWEST_POINT_OF_CHAIN_METERS = Units.inchesToMeters(
    28.25
  );

  // Speaker Translation2d
  public static final Translation2d BLUE_SPEAKER_TRANS = new Translation2d(
    0,
    5.547868
  );
  public static final Translation2d RED_SPEAKER_TRANS = new Translation2d(
    16.579342,
    5.547868
  );

  // Amp Translation2d
  public static final Translation2d BLUE_AMP_TRANS = new Translation2d(1.85, 8.20);

  public static final Translation2d RED_AMP_TRANS = new Translation2d(14.70, 8.20);

  // April Tag IDs
  public static final int BLUE_SOURCE_LEFT = 1;
  public static final int BLUE_SOURCE_RIGHT = 2;

  public static final int RED_SUBWOOFER = 3;
  public static final int RED_SPEAKER = 4;
  public static final int RED_AMP = 5;

  public static final int BLUE_AMP = 6;
  public static final int BLUE_SPEAKER = 7;
  public static final int BLUE_SUBWOOFER = 8;

  public static final int RED_SOURCE_LEFT = 9;
  public static final int RED_SOURCE_RIGHT = 10;

  // Named with Direction Facing
  public static final int RED_STAGE_SOURCE = 11;
  public static final int RED_STAGE_AMP = 12;
  public static final int RED_STAGE_STAGE = 13;

  public static final int BLUE_STAGE_STAGE = 14;
  public static final int BLUE_STAGE_AMP = 15;
  public static final int BLUE_STAGE_SOURCE = 16;

  public static int getSourceLeftAprilTagId() {
    if (!IsBlueAllience()) {
      return RED_SOURCE_LEFT;
    }
    return BLUE_SOURCE_LEFT;
  }

  public static int getSourceRightAprilTagId() {
    if (!IsBlueAllience()) {
      return RED_SOURCE_RIGHT;
    }
    return BLUE_SOURCE_RIGHT;
  }

  public static int getSubwooferAprilTagId() {
    if (!IsBlueAllience()) {
      return RED_SUBWOOFER;
    }
    return BLUE_SUBWOOFER;
  }

  public static int getSpeakerAprilTagId() {
    if (!IsBlueAllience()) {
      return RED_SPEAKER;
    }
    return BLUE_SPEAKER;
  }

  public static int getAmpAprilTagId() {
    if (!IsBlueAllience()) {
      return RED_AMP;
    }
    return BLUE_AMP;
  }

  public static int getStageAmpAprilTagId() {
    if (!IsBlueAllience()) {
      return RED_STAGE_AMP;
    }
    return BLUE_STAGE_AMP;
  }

  public static int getStageSourceAprilTagId() {
    if (!IsBlueAllience()) {
      return RED_STAGE_SOURCE;
    }
    return BLUE_STAGE_SOURCE;
  }

  public static int getStageStageAprilTagId() {
    if (!IsBlueAllience()) {
      return RED_STAGE_STAGE;
    }
    return BLUE_STAGE_STAGE;
  }

  // Tag Coordinates
  public static Pose3d getLeftSourceTag() {
    return FIELD_LAYOUT.getTagPose(getSourceLeftAprilTagId()).get();
  }

  public static Pose3d getRightSourceTag() {
    return FIELD_LAYOUT.getTagPose(getSourceRightAprilTagId()).get();
  }

  public static Pose3d getSubwooferTag() {
    return FIELD_LAYOUT.getTagPose(getSubwooferAprilTagId()).get();
  }

  public static Pose3d getSpeakerTag() {
    return FIELD_LAYOUT.getTagPose(getSpeakerAprilTagId()).get();
  }

  public static Pose3d getAmpTag() {
    return FIELD_LAYOUT.getTagPose(getAmpAprilTagId()).get();
  }

  public static Pose3d getStageSourceTag() {
    return FIELD_LAYOUT.getTagPose(getStageSourceAprilTagId()).get();
  }

  public static Pose3d getStageAmpTag() {
    return FIELD_LAYOUT.getTagPose(getStageAmpAprilTagId()).get();
  }

  public static Pose3d getStageStageTag() {
    return FIELD_LAYOUT.getTagPose(getStageStageAprilTagId()).get();
  }
}