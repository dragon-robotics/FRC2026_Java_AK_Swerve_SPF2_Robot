package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.PoseSolveStrategy;
import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import frc.robot.subsystems.vision.VisionRuntimeConfig.TagDistanceConfidenceMode;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

class VisionFilterTest {
  private static final AprilTagFieldLayout LAYOUT =
      new AprilTagFieldLayout(
          List.of(
              new AprilTag(1, new Pose3d(1.0, 1.0, 1.0, new Rotation3d())),
              new AprilTag(2, new Pose3d(2.0, 1.0, 1.0, new Rotation3d())),
              new AprilTag(3, new Pose3d(3.0, 1.0, 1.0, new Rotation3d(0.0, 0.0, Math.PI / 2.0)))),
          16.0,
          8.0);
  private static final CameraConfig CAMERA = new CameraConfig("test", new Transform3d(), 1.0);

  @ParameterizedTest(name = "{0}")
  @MethodSource("rejectionCases")
  void rejectionReasonEnforcesEachHardGateAtItsInclusiveBoundary(
      String name,
      PoseObservation observation,
      double pitchDegrees,
      double rollDegrees,
      boolean disabled,
      Pose2d referencePose,
      Optional<String> expectedReason) {
    assertEquals(
        expectedReason,
        VisionFilter.rejectionReason(
            observation, LAYOUT, pitchDegrees, rollDegrees, disabled, referencePose, config(true)));
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("normalizedDistanceCases")
  void standardDeviationsNormalizesNonpositiveAndNaNDistancesToFivePointFiveMeters(
      String name, double rawDistance) {
    Matrix<?, ?> standardDeviations =
        VisionFilter.standardDeviations(
            observation(2, 4.0, 3.0, 0.0, 0.0, rawDistance, new int[] {1, 3}),
            CAMERA,
            false,
            LAYOUT,
            config(false));

    assertEquals(0.3025, standardDeviations.get(0, 0), 1e-12);
  }

  @Test
  void standardDeviationsApplyCameraAimingAndGeometryFactorsWithAHeadingSentinel() {
    Matrix<?, ?> base = standardDeviations(2, 2.0, new int[] {1, 3}, CAMERA, false, config(true));
    Matrix<?, ?> aiming = standardDeviations(2, 2.0, new int[] {1, 3}, CAMERA, true, config(true));
    Matrix<?, ?> singleTag = standardDeviations(1, 2.0, new int[] {1}, CAMERA, false, config(true));
    Matrix<?, ?> coplanar =
        standardDeviations(2, 2.0, new int[] {1, 2}, CAMERA, false, config(true));
    Matrix<?, ?> noCoplanarPenalty =
        standardDeviations(2, 2.0, new int[] {1, 2}, CAMERA, false, config(false));

    assertEquals(0.04, base.get(0, 0), 1e-12);
    assertEquals(0.024, aiming.get(0, 0), 1e-12);
    assertEquals(0.4, singleTag.get(0, 0), 1e-12);
    assertEquals(0.2, coplanar.get(0, 0), 1e-12);
    assertEquals(0.04, noCoplanarPenalty.get(0, 0), 1e-12);
    assertEquals(1e9, base.get(2, 0), 0.0);
  }

  @Test
  void standardDeviationsFloorTranslationStandardDeviationAtOneMicrometer() {
    CameraConfig nearlyZeroFactor = new CameraConfig("floor", new Transform3d(), 1e-12);

    Matrix<?, ?> standardDeviations =
        standardDeviations(2, 2.0, new int[] {1, 3}, nearlyZeroFactor, false, config(false));

    assertEquals(1e-6, standardDeviations.get(0, 0), 0.0);
    assertEquals(1e-6, standardDeviations.get(1, 0), 0.0);
  }

  private static Stream<Arguments> rejectionCases() {
    return Stream.of(
        Arguments.of(
            "zero tags returns NO_TAGS",
            observation(0, 4.0, 3.0, 0.0, 0.0, 2.0, new int[0]),
            0.0,
            0.0,
            false,
            new Pose2d(),
            Optional.of("NO_TAGS")),
        Arguments.of(
            "one tag passes the count boundary",
            observation(1, 4.0, 3.0, 0.0, 0.0, 2.0, new int[] {1}),
            0.0,
            0.0,
            false,
            new Pose2d(4.0, 3.0, new edu.wpi.first.math.geometry.Rotation2d()),
            Optional.empty()),
        Arguments.of(
            "z above half a meter returns Z_OUT_OF_RANGE",
            observation(2, 4.0, 3.0, 0.500001, 0.0, 2.0, new int[] {1, 3}),
            0.0,
            0.0,
            false,
            new Pose2d(),
            Optional.of("Z_OUT_OF_RANGE")),
        Arguments.of(
            "z at half a meter passes",
            observation(2, 4.0, 3.0, 0.5, 0.0, 2.0, new int[] {1, 3}),
            0.0,
            0.0,
            false,
            new Pose2d(4.0, 3.0, new edu.wpi.first.math.geometry.Rotation2d()),
            Optional.empty()),
        Arguments.of(
            "field x below zero returns OUTSIDE_FIELD",
            observation(2, -0.001, 3.0, 0.0, 0.0, 2.0, new int[] {1, 3}),
            0.0,
            0.0,
            false,
            new Pose2d(),
            Optional.of("OUTSIDE_FIELD")),
        Arguments.of(
            "field zero boundaries are inclusive",
            observation(2, 0.0, 0.0, 0.0, 0.0, 2.0, new int[] {1, 3}),
            0.0,
            0.0,
            false,
            new Pose2d(),
            Optional.empty()),
        Arguments.of(
            "field boundaries are inclusive",
            observation(2, 16.0, 8.0, 0.0, 0.0, 2.0, new int[] {1, 3}),
            0.0,
            0.0,
            false,
            new Pose2d(16.0, 8.0, new edu.wpi.first.math.geometry.Rotation2d()),
            Optional.empty()),
        Arguments.of(
            "single tag ambiguity above point two returns HIGH_AMBIGUITY",
            observation(1, 4.0, 3.0, 0.0, 0.200001, 2.0, new int[] {1}),
            0.0,
            0.0,
            false,
            new Pose2d(4.0, 3.0, new edu.wpi.first.math.geometry.Rotation2d()),
            Optional.of("HIGH_AMBIGUITY")),
        Arguments.of(
            "single tag ambiguity at point two passes",
            observation(1, 4.0, 3.0, 0.0, 0.2, 2.0, new int[] {1}),
            0.0,
            0.0,
            false,
            new Pose2d(4.0, 3.0, new edu.wpi.first.math.geometry.Rotation2d()),
            Optional.empty()),
        Arguments.of(
            "distance above five point five returns TAG_DISTANCE_TOO_LARGE",
            observation(2, 4.0, 3.0, 0.0, 0.0, 5.500001, new int[] {1, 3}),
            0.0,
            0.0,
            false,
            new Pose2d(4.0, 3.0, new edu.wpi.first.math.geometry.Rotation2d()),
            Optional.of("TAG_DISTANCE_TOO_LARGE")),
        Arguments.of(
            "positive infinity distance returns TAG_DISTANCE_TOO_LARGE",
            observation(2, 4.0, 3.0, 0.0, 0.0, Double.POSITIVE_INFINITY, new int[] {1, 3}),
            0.0,
            0.0,
            false,
            new Pose2d(4.0, 3.0, new edu.wpi.first.math.geometry.Rotation2d()),
            Optional.of("TAG_DISTANCE_TOO_LARGE")),
        Arguments.of(
            "distance at five point five passes",
            observation(2, 4.0, 3.0, 0.0, 0.0, 5.5, new int[] {1, 3}),
            0.0,
            0.0,
            false,
            new Pose2d(4.0, 3.0, new edu.wpi.first.math.geometry.Rotation2d()),
            Optional.empty()),
        Arguments.of(
            "tilt above configured maximum returns TILT_UNSTABLE",
            observation(2, 4.0, 3.0, 0.0, 0.0, 2.0, new int[] {1, 3}),
            8.000001,
            0.0,
            false,
            new Pose2d(4.0, 3.0, new edu.wpi.first.math.geometry.Rotation2d()),
            Optional.of("TILT_UNSTABLE")),
        Arguments.of(
            "tilt at configured maximum passes",
            observation(2, 4.0, 3.0, 0.0, 0.0, 2.0, new int[] {1, 3}),
            -8.0,
            8.0,
            false,
            new Pose2d(4.0, 3.0, new edu.wpi.first.math.geometry.Rotation2d()),
            Optional.empty()),
        Arguments.of(
            "enabled innovation above two point five returns POSE_INNOVATION_TOO_LARGE",
            observation(2, 2.500001, 0.0, 0.0, 0.0, 2.0, new int[] {1, 3}),
            0.0,
            0.0,
            false,
            new Pose2d(),
            Optional.of("POSE_INNOVATION_TOO_LARGE")),
        Arguments.of(
            "enabled innovation at two point five passes",
            observation(2, 2.5, 0.0, 0.0, 0.0, 2.0, new int[] {1, 3}),
            0.0,
            0.0,
            false,
            new Pose2d(),
            Optional.empty()),
        Arguments.of(
            "disabled mode does not reject finite innovation",
            observation(2, 4.0, 3.0, 0.0, 0.0, 2.0, new int[] {1, 3}),
            0.0,
            0.0,
            true,
            new Pose2d(),
            Optional.empty()));
  }

  private static Stream<Arguments> normalizedDistanceCases() {
    return Stream.of(
        Arguments.of("zero distance", 0.0),
        Arguments.of("negative distance", -2.0),
        Arguments.of("NaN distance", Double.NaN));
  }

  private static Matrix<?, ?> standardDeviations(
      int tagCount,
      double distance,
      int[] tagIds,
      CameraConfig camera,
      boolean aiming,
      VisionRuntimeConfig config) {
    return VisionFilter.standardDeviations(
        observation(tagCount, 4.0, 3.0, 0.0, 0.0, distance, tagIds),
        camera,
        aiming,
        LAYOUT,
        config);
  }

  private static PoseObservation observation(
      int tagCount,
      double x,
      double y,
      double z,
      double ambiguity,
      double confidenceDistanceMeters,
      int[] tagIds) {
    return new PoseObservation(
        1.0,
        new Pose3d(x, y, z, new Rotation3d()),
        ambiguity,
        tagCount,
        confidenceDistanceMeters,
        PoseObservationType.PHOTONVISION,
        PoseSolveStrategy.LOWEST_AMBIGUITY,
        tagIds);
  }

  private static VisionRuntimeConfig config(boolean applyCoplanarPenalty) {
    return new VisionRuntimeConfig(
        8.0,
        applyCoplanarPenalty,
        "HYBRID",
        List.of(),
        false,
        TagDistanceConfidenceMode.ALL_TAG_AVERAGE,
        StartupStrategyOrder.CONSTRAINED_SECOND);
  }
}
