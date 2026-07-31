package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionConsensus.Candidate;
import frc.robot.subsystems.vision.VisionConsensus.RejectedCandidate;
import frc.robot.subsystems.vision.VisionConsensus.TemporalSelection;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.PoseSolveStrategy;
import java.lang.reflect.Method;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

class VisionConsensusTest {
  private static final double NOW_SECONDS = 100.0;

  @ParameterizedTest(name = "{0}")
  @MethodSource("timestampCases")
  void timestampRejectionReasonEnforcesFiniteFutureAndAgeBoundaries(
      String name, double timestampSeconds, Optional<String> expectedReason) {
    assertEquals(
        expectedReason, VisionConsensus.timestampRejectionReason(timestampSeconds, NOW_SECONDS));
  }

  @Test
  void newestValidCameraFrameSupersedesOlderFramesAndEqualTimestampsKeepTheFirstCandidate() {
    Candidate first = candidate(2, 99.980, 0.0, 0.0, 0.2);
    Candidate newer = candidate(2, 99.990, 1.0, 0.0, 0.2);
    Candidate equalTimestampLater = candidate(2, 99.990, 2.0, 0.0, 0.2);

    TemporalSelection selection =
        VisionConsensus.selectTimestampCoherent(
            List.of(first, newer, equalTimestampLater), NOW_SECONDS);

    assertEquals(List.of(newer), selection.selectedCandidates());
    assertRejected(selection.rejectedCandidates(), first, "SUPERSEDED_BY_NEWER_CAMERA_FRAME");
    assertRejected(
        selection.rejectedCandidates(), equalTimestampLater, "SUPERSEDED_BY_NEWER_CAMERA_FRAME");
  }

  @Test
  void invalidFramesNeverSuppressValidFramesFromTheSameCamera() {
    Candidate valid = candidate(0, 99.990, 0.0, 0.0, 0.2);
    Candidate future = candidate(0, 100.021, 1.0, 0.0, 0.2);
    Candidate stale = candidate(1, 99.499999999, 2.0, 0.0, 0.2);
    Candidate current = candidate(1, 99.985, 3.0, 0.0, 0.2);

    TemporalSelection selection =
        VisionConsensus.selectTimestampCoherent(
            List.of(valid, future, stale, current), NOW_SECONDS);

    assertEquals(List.of(valid, current), selection.selectedCandidates());
    assertRejected(selection.rejectedCandidates(), future, "FUTURE_TIMESTAMP");
    assertRejected(selection.rejectedCandidates(), stale, "STALE_TIMESTAMP");
    assertEquals(2, selection.rejectedCandidates().size());
  }

  @Test
  void temporalSelectionHandlesNoCandidatesOneCandidateAndTheInclusiveFortyMillisecondSpan() {
    TemporalSelection empty = VisionConsensus.selectTimestampCoherent(List.of(), NOW_SECONDS);
    Candidate first = candidate(0, 99.960, 0.0, 0.0, 0.2);
    Candidate last = candidate(1, 100.000, 1.0, 0.0, 0.2);

    TemporalSelection one = VisionConsensus.selectTimestampCoherent(List.of(first), NOW_SECONDS);
    TemporalSelection inclusive =
        VisionConsensus.selectTimestampCoherent(List.of(first, last), NOW_SECONDS);

    assertTrue(empty.selectedCandidates().isEmpty());
    assertTrue(empty.rejectedCandidates().isEmpty());
    assertEquals(List.of(first), one.selectedCandidates());
    assertTrue(one.rejectedCandidates().isEmpty());
    assertEquals(List.of(first, last), inclusive.selectedCandidates());
    assertTrue(inclusive.rejectedCandidates().isEmpty());
  }

  @Test
  void temporalSelectionRejectsCandidatesOutsideAWindowJustBeyondFortyMilliseconds() {
    Candidate older = candidate(0, 99.959999999, 0.0, 0.0, 0.2);
    Candidate newer = candidate(1, 100.000, 1.0, 0.0, 0.2);

    TemporalSelection selection =
        VisionConsensus.selectTimestampCoherent(List.of(older, newer), NOW_SECONDS);

    assertEquals(List.of(newer), selection.selectedCandidates());
    assertRejected(selection.rejectedCandidates(), older, "TIMESTAMP_CLUSTER_NOT_SELECTED");
  }

  @Test
  void temporalSelectionRejectsStaleBacklogBeforeItCanOutvoteCurrentCameras() {
    Candidate staleFront = candidate(0, 99.490, 0.0, 0.0, 0.2);
    Candidate staleRight = candidate(1, 99.495, 1.0, 0.0, 0.2);
    Candidate staleBack = candidate(2, 99.499, 2.0, 0.0, 0.2);
    Candidate currentFront = candidate(0, 99.980, 3.0, 0.0, 0.2);
    Candidate currentRight = candidate(1, 99.990, 4.0, 0.0, 0.2);

    TemporalSelection selection =
        VisionConsensus.selectTimestampCoherent(
            List.of(staleFront, staleRight, staleBack, currentFront, currentRight), NOW_SECONDS);

    assertEquals(List.of(currentFront, currentRight), selection.selectedCandidates());
    assertRejected(selection.rejectedCandidates(), staleFront, "STALE_TIMESTAMP");
    assertRejected(selection.rejectedCandidates(), staleRight, "STALE_TIMESTAMP");
    assertRejected(selection.rejectedCandidates(), staleBack, "STALE_TIMESTAMP");
  }

  @Test
  void temporalSelectionPrefersMoreDistinctCamerasBeforeNewerCaptureTimes() {
    Candidate olderFirst = candidate(0, 99.900, 0.0, 0.0, 0.2);
    Candidate olderSecond = candidate(1, 99.910, 1.0, 0.0, 0.2);
    Candidate olderThird = candidate(2, 99.920, 2.0, 0.0, 0.2);
    Candidate newerFirst = candidate(3, 99.970, 3.0, 0.0, 0.2);
    Candidate newerSecond = candidate(4, 99.980, 4.0, 0.0, 0.2);

    TemporalSelection selection =
        VisionConsensus.selectTimestampCoherent(
            List.of(olderFirst, olderSecond, olderThird, newerFirst, newerSecond), NOW_SECONDS);

    assertEquals(List.of(olderFirst, olderSecond, olderThird), selection.selectedCandidates());
    assertRejected(selection.rejectedCandidates(), newerFirst, "TIMESTAMP_CLUSTER_NOT_SELECTED");
    assertRejected(selection.rejectedCandidates(), newerSecond, "TIMESTAMP_CLUSTER_NOT_SELECTED");
  }

  @Test
  void temporalSelectionPrefersNewerWindowsWhenDistinctCameraCountsMatch() {
    Candidate olderFirst = candidate(0, 99.900, 0.0, 0.0, 0.2);
    Candidate olderSecond = candidate(1, 99.910, 1.0, 0.0, 0.2);
    Candidate newerFirst = candidate(2, 99.970, 2.0, 0.0, 0.2);
    Candidate newerSecond = candidate(3, 99.980, 3.0, 0.0, 0.2);

    TemporalSelection selection =
        VisionConsensus.selectTimestampCoherent(
            List.of(olderFirst, olderSecond, newerFirst, newerSecond), NOW_SECONDS);

    assertEquals(List.of(newerFirst, newerSecond), selection.selectedCandidates());
    assertRejected(selection.rejectedCandidates(), olderFirst, "TIMESTAMP_CLUSTER_NOT_SELECTED");
    assertRejected(selection.rejectedCandidates(), olderSecond, "TIMESTAMP_CLUSTER_NOT_SELECTED");
  }

  @Test
  void temporalComparatorPrefersLowerStdDevBeforeTheLexicographicallyLowerCameraVector()
      throws ReflectiveOperationException {
    List<Candidate> lowerStandardDeviation =
        List.of(
            candidate(5, "wide-right", 99.960, 0.0, 0.0, 0.05, 0.8),
            candidate(6, "wide-back", 100.000, 1.0, 0.0, 0.05, 0.9));
    List<Candidate> lexicographicallyLowerButNoisier =
        List.of(
            candidate(1, "front-left", 99.960, 2.0, 0.0, 0.20, 1.0),
            candidate(2, "front-right", 100.000, 3.0, 0.0, 0.20, 1.1));

    assertTrue(
        compareTemporalWindows(lowerStandardDeviation, lexicographicallyLowerButNoisier) > 0);
  }

  @Test
  void temporalComparatorPrefersTheLexicographicallyLowerSortedCameraVectorAfterQualityTies()
      throws ReflectiveOperationException {
    List<Candidate> lexicographicallyLower =
        List.of(
            candidate(1, "front-left", 99.960, 0.0, 0.0, 0.10, 0.4),
            candidate(5, "wide-right", 100.000, 1.0, 0.0, 0.10, 0.5));
    List<Candidate> lexicographicallyHigher =
        List.of(
            candidate(2, "front-right", 99.960, 2.0, 0.0, 0.10, 0.6),
            candidate(3, "wide-left", 100.000, 3.0, 0.0, 0.10, 0.7));

    assertTrue(compareTemporalWindows(lexicographicallyLower, lexicographicallyHigher) > 0);
  }

  @Test
  void spatialSelectionReturnsTheOnlyOriginalCandidateWithoutChangingItsMeasurement() {
    Candidate candidate = candidate(3, "rear-precision", 99.990, 2.0, 4.0, 0.37, 1.75);

    Candidate winner = VisionConsensus.selectSpatialConsensus(List.of(candidate)).orElseThrow();

    assertSame(candidate, winner);
    assertSame(candidate.observation(), winner.observation());
    assertSame(candidate.visionPose(), winner.visionPose());
    assertSame(candidate.standardDeviations(), winner.standardDeviations());
    assertEquals("rear-precision", winner.cameraName());
    assertEquals(99.990, winner.observation().timestampSeconds(), 0.0);
    assertEquals(0.37, winner.standardDeviations().get(0, 0), 0.0);
    assertEquals(1.75, winner.innovationMeters(), 0.0);
  }

  @Test
  void spatialSelectionPrefersTheLargestInclusiveNeighborhood() {
    Candidate left = candidate(0, 99.980, 0.0, 0.0, 0.2);
    Candidate center = candidate(1, 99.985, 0.45, 0.0, 0.2);
    Candidate right = candidate(2, 99.990, 0.90, 0.0, 0.2);

    Optional<Candidate> winner =
        VisionConsensus.selectSpatialConsensus(List.of(left, center, right));

    assertEquals(Optional.of(center), winner);
  }

  @Test
  void spatialSelectionExcludesCandidatesJustBeyondTheInclusiveRadius() {
    Candidate lowerQualityCamera = candidate(0, 99.980, 0.0, 0.0, 0.4);
    Candidate higherQualityCamera = candidate(1, 99.990, 0.450000001, 0.0, 0.1);

    Candidate winner =
        VisionConsensus.selectSpatialConsensus(List.of(lowerQualityCamera, higherQualityCamera))
            .orElseThrow();

    assertSame(higherQualityCamera, winner);
  }

  @Test
  void spatialSelectionUsesReferenceQualityScoreAfterNeighborhoodSize() {
    Candidate noisier = candidate(4, 99.980, 0.0, 0.0, 0.2);
    Candidate cleaner = candidate(7, 99.990, 0.10, 0.0, 0.1);

    Candidate winner =
        VisionConsensus.selectSpatialConsensus(List.of(noisier, cleaner)).orElseThrow();

    assertSame(cleaner, winner);
  }

  @Test
  void spatialSelectionUsesLowerCameraIndexAsTheFinalStableTieBreaker() {
    Candidate higherCameraIndex = candidate(9, 99.980, 0.0, 0.0, 0.2);
    Candidate lowerCameraIndex = candidate(1, 99.990, 0.0, 0.0, 0.2);

    Candidate winner =
        VisionConsensus.selectSpatialConsensus(List.of(higherCameraIndex, lowerCameraIndex))
            .orElseThrow();

    assertSame(lowerCameraIndex, winner);
  }

  private static Stream<Arguments> timestampCases() {
    return Stream.of(
        Arguments.of("NaN timestamp", Double.NaN, Optional.of("INVALID_TIMESTAMP")),
        Arguments.of(
            "positive infinity timestamp",
            Double.POSITIVE_INFINITY,
            Optional.of("INVALID_TIMESTAMP")),
        Arguments.of(
            "negative infinity timestamp",
            Double.NEGATIVE_INFINITY,
            Optional.of("INVALID_TIMESTAMP")),
        Arguments.of("future boundary", 100.020, Optional.empty()),
        Arguments.of("age boundary", 99.500, Optional.empty()),
        Arguments.of(
            "one nanosecond beyond future boundary",
            100.020000001,
            Optional.of("FUTURE_TIMESTAMP")),
        Arguments.of(
            "one nanosecond beyond age boundary", 99.499999999, Optional.of("STALE_TIMESTAMP")));
  }

  private static Candidate candidate(
      int cameraIndex,
      String cameraName,
      double timestampSeconds,
      double xMeters,
      double yMeters,
      double linearStdDevMeters,
      double innovationMeters) {
    PoseObservation observation =
        new PoseObservation(
            timestampSeconds,
            new Pose3d(xMeters, yMeters, 0.0, new Rotation3d()),
            0.0,
            2,
            2.0,
            PoseObservationType.PHOTONVISION,
            PoseSolveStrategy.LOWEST_AMBIGUITY,
            new int[] {1, 2});
    Matrix<N3, N1> standardDeviations =
        VecBuilder.fill(linearStdDevMeters, linearStdDevMeters, VisionConstants.HEADING_STD_DEV);
    return new Candidate(
        cameraIndex,
        cameraName,
        observation,
        new Pose2d(xMeters, yMeters, Rotation2d.kZero),
        standardDeviations,
        innovationMeters);
  }

  private static Candidate candidate(
      int cameraIndex,
      double timestampSeconds,
      double xMeters,
      double yMeters,
      double linearStdDevMeters) {
    return candidate(
        cameraIndex,
        "camera-" + cameraIndex,
        timestampSeconds,
        xMeters,
        yMeters,
        linearStdDevMeters,
        0.0);
  }

  private static int compareTemporalWindows(List<Candidate> first, List<Candidate> second)
      throws ReflectiveOperationException {
    Method method =
        VisionConsensus.class.getDeclaredMethod("compareTemporalWindows", List.class, List.class);
    method.setAccessible(true);
    return (int) method.invoke(null, first, second);
  }

  private static void assertRejected(
      List<RejectedCandidate> rejectedCandidates, Candidate candidate, String reason) {
    assertTrue(
        rejectedCandidates.stream()
            .anyMatch(
                rejected -> rejected.candidate() == candidate && rejected.reason().equals(reason)));
  }
}
