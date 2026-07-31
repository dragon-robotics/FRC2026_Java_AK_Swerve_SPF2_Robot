package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.Optional;

/** Hard acceptance gates and covariance selection for vision pose observations. */
public final class VisionFilter {
  public static final String NO_TAGS = "NO_TAGS";
  public static final String Z_OUT_OF_RANGE = "Z_OUT_OF_RANGE";
  public static final String OUTSIDE_FIELD = "OUTSIDE_FIELD";
  public static final String HIGH_AMBIGUITY = "HIGH_AMBIGUITY";
  public static final String TAG_DISTANCE_TOO_LARGE = "TAG_DISTANCE_TOO_LARGE";
  public static final String TILT_UNSTABLE = "TILT_UNSTABLE";
  public static final String POSE_INNOVATION_TOO_LARGE = "POSE_INNOVATION_TOO_LARGE";

  private VisionFilter() {}

  public static Optional<String> rejectionReason(
      PoseObservation observation,
      AprilTagFieldLayout layout,
      double pitchDegrees,
      double rollDegrees,
      boolean disabled,
      Pose2d referencePose,
      VisionRuntimeConfig config) {
    if (observation.tagCount() == 0) {
      return Optional.of(NO_TAGS);
    }
    if (Math.abs(observation.pose().getZ()) > VisionConstants.MAX_Z_ERROR_METERS) {
      return Optional.of(Z_OUT_OF_RANGE);
    }
    if (observation.pose().getX() < 0.0
        || observation.pose().getX() > layout.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > layout.getFieldWidth()) {
      return Optional.of(OUTSIDE_FIELD);
    }
    if (observation.tagCount() == 1 && observation.ambiguity() > VisionConstants.MAX_AMBIGUITY) {
      return Optional.of(HIGH_AMBIGUITY);
    }
    if (observation.confidenceDistanceMeters() > VisionConstants.MAX_CONFIDENCE_DISTANCE_METERS) {
      return Optional.of(TAG_DISTANCE_TOO_LARGE);
    }
    if (Math.abs(pitchDegrees) > config.maxAbsTiltDegrees()
        || Math.abs(rollDegrees) > config.maxAbsTiltDegrees()) {
      return Optional.of(TILT_UNSTABLE);
    }
    if (!disabled
        && observation
                .pose()
                .toPose2d()
                .getTranslation()
                .getDistance(referencePose.getTranslation())
            > VisionConstants.MAX_POSE_INNOVATION_METERS) {
      return Optional.of(POSE_INNOVATION_TOO_LARGE);
    }
    return Optional.empty();
  }

  public static Matrix<N3, N1> standardDeviations(
      PoseObservation observation,
      CameraConfig camera,
      boolean aiming,
      AprilTagFieldLayout layout,
      VisionRuntimeConfig config) {
    double effectiveDistance =
        observation.confidenceDistanceMeters() > 0.0
            ? observation.confidenceDistanceMeters()
            : VisionConstants.MAX_CONFIDENCE_DISTANCE_METERS;
    double translationStandardDeviation =
        VisionConstants.LINEAR_STD_DEV_BASELINE
            * effectiveDistance
            * effectiveDistance
            / observation.tagCount()
            * camera.stdDevFactor();
    if (aiming) {
      translationStandardDeviation *= VisionConstants.AIMING_STD_DEV_FACTOR;
    }
    if (observation.tagCount() == 1
        || (config.applyCoplanarPenalty()
            && VisionGeometry.areTagsCoplanar(layout, observation.tagIds()))) {
      translationStandardDeviation *= VisionConstants.VULNERABLE_GEOMETRY_STD_DEV_FACTOR;
    }
    translationStandardDeviation = Math.max(1e-6, translationStandardDeviation);
    return VecBuilder.fill(
        translationStandardDeviation,
        translationStandardDeviation,
        VisionConstants.HEADING_STD_DEV);
  }
}
