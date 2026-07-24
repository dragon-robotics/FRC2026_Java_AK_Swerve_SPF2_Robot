package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants.FieldZones;
import org.junit.jupiter.api.Test;

class FieldConstantsTest {
  @Test
  void fieldZonesClassifiesAllianceLeftAndRightFromPose() {
    assertEquals(
        FieldZones.ALLIANCE_LEFT,
        FieldZones.fromPose(
            new Pose2d(0.0, FieldConstants.FIELD_WIDTH * 0.75, Rotation2d.kZero),
            Alliance.Blue));
    assertEquals(
        FieldZones.ALLIANCE_RIGHT,
        FieldZones.fromPose(
            new Pose2d(0.0, FieldConstants.FIELD_WIDTH * 0.25, Rotation2d.kZero),
            Alliance.Blue));
  }

  @Test
  void fieldZonesMirrorsRedAlliancePoseBeforeClassification() {
    assertEquals(
        FieldZones.ALLIANCE_LEFT,
        FieldZones.fromPose(
            new Pose2d(
                FieldConstants.FIELD_LENGTH,
                FieldConstants.FIELD_WIDTH * 0.25,
                Rotation2d.kZero),
            Alliance.Red));
  }
}
