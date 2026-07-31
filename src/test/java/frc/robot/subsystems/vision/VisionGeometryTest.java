package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.constants.FieldConstants;
import java.util.List;
import org.junit.jupiter.api.Test;

class VisionGeometryTest {
  @Test
  void coplanarityUsesLocalPositiveXInsteadOfLocalPositiveZ() {
    AprilTagFieldLayout layout =
        layout(
            new AprilTag(1, new Pose3d(0.0, 0.0, 0.0, new Rotation3d())),
            new AprilTag(2, new Pose3d(1.0, 0.0, 0.0, new Rotation3d(Math.PI / 2.0, 0.0, 0.0))));

    assertTrue(VisionGeometry.areTagsCoplanar(layout, new int[] {1, 2}));
  }

  @Test
  void coplanarityAcceptsNormalsAtTheFifteenDegreeBoundary() {
    AprilTagFieldLayout layout =
        layout(
            tag(1, new Rotation3d()),
            tag(2, new Rotation3d(0.0, 0.0, Units.degreesToRadians(15.0))));

    assertTrue(VisionGeometry.areTagsCoplanar(layout, new int[] {1, 2}));
  }

  @Test
  void coplanarityRejectsNormalsJustPastTheFifteenDegreeBoundary() {
    AprilTagFieldLayout layout =
        layout(
            tag(1, new Rotation3d()),
            tag(2, new Rotation3d(0.0, 0.0, Units.degreesToRadians(15.001))));

    assertFalse(VisionGeometry.areTagsCoplanar(layout, new int[] {1, 2}));
  }

  @Test
  void coplanarityRejectsOppositeTagNormals() {
    AprilTagFieldLayout layout =
        layout(tag(1, new Rotation3d()), tag(2, new Rotation3d(0.0, 0.0, Math.PI)));

    assertFalse(VisionGeometry.areTagsCoplanar(layout, new int[] {1, 2}));
  }

  @Test
  void oneKnownTagAndUnknownTagsAreConservativelyCoplanar() {
    AprilTagFieldLayout layout = layout(tag(1, new Rotation3d()));

    assertTrue(VisionGeometry.areTagsCoplanar(layout, new int[] {1}));
    assertTrue(VisionGeometry.areTagsCoplanar(layout, new int[] {99, 1}));
    assertTrue(VisionGeometry.areTagsCoplanar(layout, new int[] {1, 99}));
  }

  @Test
  void angleBetweenTagNormalsUsesPositiveX() {
    double angle =
        VisionGeometry.angleBetweenTagNormalsRadians(
            new Rotation3d(), new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0)));

    assertEquals(30.0, Units.radiansToDegrees(angle), 1e-9);
  }

  @Test
  void productionLayoutKeepsTagsOnOneHubFaceCoplanar() {
    assertTrue(
        VisionGeometry.areTagsCoplanar(FieldConstants.APTAG_FIELD_LAYOUT, new int[] {25, 26}));
  }

  @Test
  void productionLayoutSeparatesTagsOnDifferentHubFaces() {
    assertFalse(
        VisionGeometry.areTagsCoplanar(FieldConstants.APTAG_FIELD_LAYOUT, new int[] {26, 27}));
  }

  private static AprilTag tag(int id, Rotation3d rotation) {
    return new AprilTag(id, new Pose3d(id, 0.0, 0.0, rotation));
  }

  private static AprilTagFieldLayout layout(AprilTag... tags) {
    return new AprilTagFieldLayout(List.of(tags), 16.0, 8.0);
  }
}
