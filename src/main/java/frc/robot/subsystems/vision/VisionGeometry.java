package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Geometry helpers for evaluating the orientation of observed AprilTags. */
public final class VisionGeometry {
  private static final double COPLANAR_ANGLE_THRESHOLD_RADIANS =
      Units.degreesToRadians(VisionConstants.COPLANAR_ANGLE_THRESHOLD_DEGREES);
  private static final Translation3d TAG_NORMAL = new Translation3d(1.0, 0.0, 0.0);

  private VisionGeometry() {}

  public static boolean areTagsCoplanar(AprilTagFieldLayout layout, int[] tagIds) {
    if (tagIds == null || tagIds.length < 2) {
      return true;
    }

    var firstTagPose = layout.getTagPose(tagIds[0]);
    if (firstTagPose.isEmpty()) {
      return true;
    }

    Rotation3d firstRotation = firstTagPose.get().getRotation();
    for (int index = 1; index < tagIds.length; index++) {
      var tagPose = layout.getTagPose(tagIds[index]);
      if (tagPose.isPresent()
          && angleBetweenTagNormalsRadians(firstRotation, tagPose.get().getRotation())
              > COPLANAR_ANGLE_THRESHOLD_RADIANS) {
        return false;
      }
    }
    return true;
  }

  public static double angleBetweenTagNormalsRadians(Rotation3d first, Rotation3d second) {
    Translation3d firstNormal = TAG_NORMAL.rotateBy(first);
    Translation3d secondNormal = TAG_NORMAL.rotateBy(second);
    double cosine =
        firstNormal.dot(secondNormal) / (firstNormal.getNorm() * secondNormal.getNorm());
    return Math.acos(Math.max(-1.0, Math.min(1.0, cosine)));
  }
}
