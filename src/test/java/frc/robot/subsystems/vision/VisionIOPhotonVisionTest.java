package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.PoseSolveStrategy;
import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import frc.robot.subsystems.vision.VisionRuntimeConfig.TagDistanceConfidenceMode;
import frc.robot.util.constants.FieldConstants;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

class VisionIOPhotonVisionTest {
  private static final CameraConfig CAMERA =
      new CameraConfig("test-camera", new Transform3d(), 1.0);
  private static final double NOW_SECONDS = 100.0;

  @Test
  void drainsEveryUnreadFrameInCaptureOrderAndStopsAfterEachFirstSuccess() {
    PhotonTrackedTarget one = target(1, 4.0, 0.1, 3.0, -2.0);
    PhotonTrackedTarget two = target(2, 3.0, 0.2, 4.0, -3.0);
    PhotonTrackedTarget duplicateTwo = target(2, 2.0, 0.3, 5.0, -4.0);
    PhotonTrackedTarget three = target(3, 1.0, 0.4, 12.5, -6.25);
    PhotonPipelineResult first = result(1.25, one);
    PhotonPipelineResult second = result(2.5, two, duplicateTwo);
    PhotonPipelineResult third = result(3.75, three);
    FakeCameraSource source = new FakeCameraSource(List.of(List.of(first, second, third)));
    ScriptedVisionIO io = scriptedIo(source, config(TagDistanceConfidenceMode.ALL_TAG_AVERAGE));
    io.script(
        first, PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, estimate(first, Pose3d.kZero, one));
    io.script(
        second,
        PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
        estimate(second, new Pose3d(2.0, 0.0, 0.0, Rotation3d.kZero), two, duplicateTwo));
    io.script(
        third,
        PoseSolveStrategy.LOWEST_AMBIGUITY,
        estimate(third, new Pose3d(3.0, 0.0, 0.0, Rotation3d.kZero), three));
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    io.updateInputs(inputs, 2.5);

    PoseObservation[] observations = inputs.getPoseObservations();
    assertAll(
        () -> assertEquals(3, observations.length),
        () -> assertEquals(1.25, observations[0].timestampSeconds(), 1e-12),
        () -> assertEquals(2.5, observations[1].timestampSeconds(), 1e-12),
        () -> assertEquals(3.75, observations[2].timestampSeconds(), 1e-12),
        () ->
            assertEquals(
                PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, observations[0].strategy()),
        () -> assertEquals(PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE, observations[1].strategy()),
        () -> assertEquals(PoseSolveStrategy.LOWEST_AMBIGUITY, observations[2].strategy()),
        () ->
            assertEquals(
                PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, observations[0].type()),
        () -> assertEquals(PoseObservationType.PHOTONVISION, observations[1].type()),
        () -> assertEquals(PoseObservationType.PHOTONVISION, observations[2].type()),
        () -> assertEquals(12.5, inputs.latestTargetObservation.tx().getDegrees(), 1e-12),
        () -> assertEquals(-6.25, inputs.latestTargetObservation.ty().getDegrees(), 1e-12),
        () -> assertArrayEquals(new int[] {1}, observations[0].tagIds()),
        () -> assertArrayEquals(new int[] {2, 2}, observations[1].tagIds()),
        () -> assertArrayEquals(new int[] {3}, observations[2].tagIds()),
        () -> assertArrayEquals(new int[] {1, 2, 3}, inputs.getTagIds()),
        () ->
            assertEquals(
                List.of(
                    PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
                    PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    PoseSolveStrategy.LOWEST_AMBIGUITY),
                io.attempts()));
  }

  @Test
  void overwritesTransientFieldsAndDrainsBufferedFramesWhileDisconnected() {
    PhotonTrackedTarget target = target(7, 2.0, 0.1, 9.0, -8.0);
    PhotonPipelineResult populated = result(4.0, target);
    FakeCameraSource source = new FakeCameraSource(List.of(List.of(populated), List.of()));
    ScriptedVisionIO io = scriptedIo(source, config(TagDistanceConfidenceMode.ALL_TAG_AVERAGE));
    io.script(
        populated, PoseSolveStrategy.LOWEST_AMBIGUITY, estimate(populated, Pose3d.kZero, target));
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    io.updateInputs(inputs, 4.0);
    io.updateInputs(inputs, 4.0);

    assertAll(
        () -> assertEquals(Rotation2d.kZero, inputs.latestTargetObservation.tx()),
        () -> assertEquals(Rotation2d.kZero, inputs.latestTargetObservation.ty()),
        () -> assertEquals(0, inputs.getPoseObservations().length),
        () -> assertEquals(0, inputs.poseObservationTagIds.length),
        () -> assertArrayEquals(new int[0], inputs.getTagIds()));

    PhotonPipelineResult buffered = result(5.0, target);
    FakeCameraSource disconnected = new FakeCameraSource(List.of(List.of(buffered)));
    disconnected.connected = false;
    ScriptedVisionIO disconnectedIo =
        scriptedIo(disconnected, config(TagDistanceConfidenceMode.ALL_TAG_AVERAGE));
    disconnectedIo.script(
        buffered, PoseSolveStrategy.LOWEST_AMBIGUITY, estimate(buffered, Pose3d.kZero, target));

    disconnectedIo.updateInputs(inputs, 5.0);

    assertAll(
        () -> assertFalse(inputs.connected),
        () -> assertEquals(1, inputs.getPoseObservations().length),
        () -> assertArrayEquals(new int[] {7}, inputs.getTagIds()),
        () -> assertEquals(1, disconnected.drainCalls));
  }

  @Test
  void lastEmptyFrameClearsTargetAnglesWithoutDiscardingEarlierObservation() {
    PhotonTrackedTarget target = target(15, 2.0, 0.1, 9.0, -8.0);
    PhotonPipelineResult populated = result(5.25, target);
    PhotonPipelineResult empty = result(5.5);
    FakeCameraSource source = new FakeCameraSource(List.of(List.of(populated, empty)));
    ScriptedVisionIO io = scriptedIo(source, config(TagDistanceConfidenceMode.ALL_TAG_AVERAGE));
    io.script(
        populated, PoseSolveStrategy.LOWEST_AMBIGUITY, estimate(populated, Pose3d.kZero, target));
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    io.updateInputs(inputs, 5.5);

    assertAll(
        () -> assertEquals(Rotation2d.kZero, inputs.latestTargetObservation.tx()),
        () -> assertEquals(Rotation2d.kZero, inputs.latestTargetObservation.ty()),
        () -> assertEquals(1, inputs.getPoseObservations().length),
        () -> assertArrayEquals(new int[] {15}, inputs.getTagIds()));
  }

  @Test
  void lastNullFrameClearsTargetAnglesWithoutDiscardingEarlierObservation() {
    PhotonTrackedTarget target = target(16, 2.0, 0.1, 7.0, -6.0);
    PhotonPipelineResult populated = result(5.75, target);
    List<PhotonPipelineResult> frames = new ArrayList<>();
    frames.add(populated);
    frames.add(null);
    FakeCameraSource source = new FakeCameraSource(List.of(frames));
    ScriptedVisionIO io = scriptedIo(source, config(TagDistanceConfidenceMode.ALL_TAG_AVERAGE));
    io.script(
        populated, PoseSolveStrategy.LOWEST_AMBIGUITY, estimate(populated, Pose3d.kZero, target));
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    io.updateInputs(inputs, 5.75);

    assertAll(
        () -> assertEquals(Rotation2d.kZero, inputs.latestTargetObservation.tx()),
        () -> assertEquals(Rotation2d.kZero, inputs.latestTargetObservation.ty()),
        () -> assertEquals(1, inputs.getPoseObservations().length),
        () -> assertArrayEquals(new int[] {16}, inputs.getTagIds()));
  }

  @Test
  void observationArithmeticFiltersIdsClampsAmbiguityAndUsesSelectedDistanceMode() {
    PhotonTrackedTarget negative = target(-1, 1.0, 0.9, 0.0, 0.0);
    PhotonTrackedTarget zero = target(0, 2.0, 0.8, 0.0, 0.0);
    PhotonTrackedTarget noTransform = target(4, null, 0.3, 0.0, 0.0);
    PhotonTrackedTarget threeMeters = target(5, 3.0, -0.2, 0.0, 0.0);
    PhotonTrackedTarget fiveMeters = target(6, 5.0, 0.6, 0.0, 0.0);
    PhotonPipelineResult result = result(6.0, negative, zero, noTransform, threeMeters, fiveMeters);

    PoseObservation average = observe(result, TagDistanceConfidenceMode.ALL_TAG_AVERAGE);
    PoseObservation maximum = observe(result, TagDistanceConfidenceMode.MAX_TAG_DISTANCE);

    assertAll(
        () -> assertArrayEquals(new int[] {4, 5, 6}, average.tagIds()),
        () -> assertEquals(3, average.tagCount()),
        () -> assertEquals(0.3, average.ambiguity(), 1e-12),
        () -> assertEquals(4.0, average.confidenceDistanceMeters(), 1e-12),
        () -> assertEquals(5.0, maximum.confidenceDistanceMeters(), 1e-12));
  }

  @Test
  void noUsableDistanceIsInfiniteAndCoarseAllFarCheckUsesInclusiveEightMeterBoundary() {
    PhotonTrackedTarget noTransform = target(8, null, 0.1, 0.0, 0.0);
    PhotonTrackedTarget prefilterOnly = target(14, 1.0, 0.9, 0.0, 0.0);
    PhotonPipelineResult missing = result(7.0, noTransform, prefilterOnly);
    FakeCameraSource missingSource = new FakeCameraSource(List.of(List.of(missing)));
    ScriptedVisionIO missingIo =
        scriptedIo(missingSource, config(TagDistanceConfidenceMode.ALL_TAG_AVERAGE));
    missingIo.script(
        missing, PoseSolveStrategy.LOWEST_AMBIGUITY, estimate(missing, Pose3d.kZero, noTransform));
    VisionIO.VisionIOInputs missingInputs = new VisionIO.VisionIOInputs();
    missingIo.updateInputs(missingInputs, 7.0);
    PoseObservation observation = missingInputs.getPoseObservations()[0];
    assertAll(
        () -> assertEquals(Double.POSITIVE_INFINITY, observation.confidenceDistanceMeters()),
        () -> assertArrayEquals(new int[] {8}, observation.tagIds()),
        () -> assertArrayEquals(new int[] {8}, missingInputs.getTagIds()));

    PhotonTrackedTarget atBoundary = target(9, 8.0, 0.1, 0.0, 0.0);
    PhotonTrackedTarget beyond = target(10, 8.000_001, 0.1, 0.0, 0.0);
    PhotonPipelineResult allowed = result(8.0, noTransform, beyond, atBoundary);
    PhotonTrackedTarget skippedNull = target(12, null, 0.1, 0.0, 0.0);
    PhotonTrackedTarget skippedFar = target(11, 8.000_001, 0.1, 0.0, 0.0);
    PhotonPipelineResult skipped = result(9.0, skippedNull, skippedFar);
    FakeCameraSource source = new FakeCameraSource(List.of(List.of(allowed, skipped)));
    ScriptedVisionIO io = scriptedIo(source, config(TagDistanceConfidenceMode.ALL_TAG_AVERAGE));
    io.script(
        allowed,
        PoseSolveStrategy.LOWEST_AMBIGUITY,
        estimate(allowed, Pose3d.kZero, noTransform, beyond, atBoundary));
    io.script(
        skipped,
        PoseSolveStrategy.LOWEST_AMBIGUITY,
        estimate(skipped, Pose3d.kZero, noTransform, beyond));
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    io.updateInputs(inputs, 8.0);

    assertAll(
        () -> assertEquals(1, inputs.getPoseObservations().length),
        () -> assertArrayEquals(new int[] {8, 10, 9}, inputs.getTagIds()),
        () ->
            assertEquals(
                List.of(
                    PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
                    PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    PoseSolveStrategy.LOWEST_AMBIGUITY),
                io.attempts()));
  }

  @Test
  void eligibleFrameWithNoSuccessfulSolverDoesNotContributeCameraTagIds() {
    PhotonTrackedTarget target = target(17, 2.0, 0.1, 6.0, -4.0);
    PhotonPipelineResult result = result(9.5, target);
    FakeCameraSource source = new FakeCameraSource(List.of(List.of(result)));
    ScriptedVisionIO io = scriptedIo(source, config(TagDistanceConfidenceMode.ALL_TAG_AVERAGE));
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    io.updateInputs(inputs, 9.5);

    assertAll(
        () -> assertEquals(6.0, inputs.latestTargetObservation.tx().getDegrees(), 1e-12),
        () -> assertEquals(-4.0, inputs.latestTargetObservation.ty().getDegrees(), 1e-12),
        () -> assertEquals(0, inputs.getPoseObservations().length),
        () -> assertArrayEquals(new int[0], inputs.getTagIds()),
        () ->
            assertEquals(
                List.of(
                    PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
                    PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    PoseSolveStrategy.LOWEST_AMBIGUITY),
                io.attempts()));
  }

  @Test
  void directSolverAttemptsCallTheRequiredEstimatorEntryPoints() {
    PhotonPipelineResult result = result(10.0, target(11, 1.0, 0.1, 0.0, 0.0));
    RecordingEstimator estimator = new RecordingEstimator();
    estimator.multiResult = Optional.of(estimate(result, Pose3d.kZero, result.getBestTarget()));
    estimator.lowestResult = Optional.of(estimate(result, Pose3d.kZero, result.getBestTarget()));
    ProbeHeadingProvider heading = new ProbeHeadingProvider(0.0, 0.0);
    VisionIOPhotonVision io = io(new FakeCameraSource(List.of()), estimator, heading);

    assertTrue(
        io.attemptStrategy(PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, result).isPresent());
    assertTrue(io.attemptStrategy(PoseSolveStrategy.LOWEST_AMBIGUITY, result).isPresent());

    assertEquals(List.of("multi", "lowest"), estimator.events);
  }

  @Test
  void trigRequiresInclusiveRateAndTimestampedHeadingBeforeEstimate() {
    PhotonPipelineResult result = result(11.25, target(12, 1.0, 0.1, 0.0, 0.0));
    RecordingEstimator estimator = new RecordingEstimator();
    estimator.trigResult = Optional.of(estimate(result, Pose3d.kZero, result.getBestTarget()));
    ProbeHeadingProvider heading = new ProbeHeadingProvider(0.0, 1.0);
    heading.heading = Optional.of(Rotation2d.fromDegrees(37.0));
    VisionIOPhotonVision io = io(new FakeCameraSource(List.of()), estimator, heading);

    assertTrue(io.attemptStrategy(PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE, result).isPresent());
    assertAll(
        () -> assertEquals(List.of(11.25), heading.headingTimestamps),
        () -> assertEquals(List.of(11.25), estimator.headingDataTimestamps),
        () -> assertEquals(List.of(Rotation2d.fromDegrees(37.0)), estimator.headingData),
        () -> assertEquals(List.of("heading", "trig"), estimator.events));

    heading.angularRate = -1.0;
    heading.headingTimestamps.clear();
    estimator.headingDataTimestamps.clear();
    estimator.headingData.clear();
    estimator.events.clear();
    assertTrue(io.attemptStrategy(PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE, result).isPresent());
    assertAll(
        () -> assertEquals(List.of(11.25), heading.headingTimestamps),
        () -> assertEquals(List.of(11.25), estimator.headingDataTimestamps),
        () -> assertEquals(List.of(Rotation2d.fromDegrees(37.0)), estimator.headingData),
        () -> assertEquals(List.of("heading", "trig"), estimator.events));

    heading.angularRate = 1.000_001;
    estimator.events.clear();
    assertTrue(io.attemptStrategy(PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE, result).isEmpty());
    assertTrue(estimator.events.isEmpty());

    heading.angularRate = -1.000_001;
    assertTrue(io.attemptStrategy(PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE, result).isEmpty());
    assertTrue(estimator.events.isEmpty());

    heading.angularRate = 0.0;
    heading.heading = Optional.empty();
    assertTrue(io.attemptStrategy(PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE, result).isEmpty());
    assertTrue(
        io(new FakeCameraSource(List.of()), estimator, null)
            .attemptStrategy(PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE, result)
            .isEmpty());
  }

  @Test
  void constrainedRequiresAllPrerequisitesAndUsesLowestSeedBeforeTimestampedFallback() {
    PhotonPipelineResult result = result(12.5, target(13, 1.0, 0.1, 0.0, 0.0));
    RecordingEstimator estimator = new RecordingEstimator();
    Pose3d lowestSeed = new Pose3d(1.0, 2.0, 0.0, Rotation3d.kZero);
    Pose3d drivetrainSeed = new Pose3d(3.0, 4.0, 0.0, Rotation3d.kZero);
    estimator.lowestResult = Optional.of(estimate(result, lowestSeed, result.getBestTarget()));
    estimator.constrainedResult =
        Optional.of(estimate(result, Pose3d.kZero, result.getBestTarget()));
    ProbeHeadingProvider heading = new ProbeHeadingProvider(0.0, 0.5);
    heading.heading = Optional.of(Rotation2d.fromDegrees(15.0));
    heading.seed = Optional.of(drivetrainSeed);
    FakeCameraSource calibrated = new FakeCameraSource(List.of());
    calibrated.cameraMatrix = Optional.of(cameraMatrix());
    calibrated.distortion = Optional.of(distortion());
    VisionIOPhotonVision io = io(calibrated, estimator, heading);

    assertTrue(io.attemptStrategy(PoseSolveStrategy.CONSTRAINED_SOLVEPNP, result).isPresent());
    assertAll(
        () -> assertEquals(List.of(12.5), heading.headingTimestamps),
        () -> assertTrue(heading.seedTimestamps.isEmpty()),
        () -> assertEquals(lowestSeed, estimator.constrainedSeed),
        () -> assertFalse(estimator.headingFree),
        () -> assertEquals(0.2, estimator.headingScale, 1e-12),
        () -> assertEquals(List.of("lowest", "heading", "constrained"), estimator.events));

    estimator.events.clear();
    estimator.lowestResult = Optional.empty();
    assertTrue(io.attemptStrategy(PoseSolveStrategy.CONSTRAINED_SOLVEPNP, result).isPresent());
    assertAll(
        () -> assertEquals(List.of(12.5), heading.seedTimestamps),
        () -> assertEquals(drivetrainSeed, estimator.constrainedSeed),
        () -> assertEquals(List.of("lowest", "heading", "constrained"), estimator.events));

    estimator.events.clear();
    heading.seed = Optional.empty();
    assertTrue(io.attemptStrategy(PoseSolveStrategy.CONSTRAINED_SOLVEPNP, result).isEmpty());
    assertEquals(List.of("lowest"), estimator.events);

    heading.seed = Optional.of(drivetrainSeed);
    heading.angularRate = -0.5;
    estimator.events.clear();
    assertTrue(io.attemptStrategy(PoseSolveStrategy.CONSTRAINED_SOLVEPNP, result).isPresent());
    assertEquals(List.of("lowest", "heading", "constrained"), estimator.events);

    heading.angularRate = -0.500_001;
    estimator.events.clear();
    assertTrue(io.attemptStrategy(PoseSolveStrategy.CONSTRAINED_SOLVEPNP, result).isEmpty());
    assertTrue(estimator.events.isEmpty());

    heading.angularRate = 0.500_001;
    assertTrue(io.attemptStrategy(PoseSolveStrategy.CONSTRAINED_SOLVEPNP, result).isEmpty());
    heading.angularRate = 0.0;
    heading.heading = Optional.empty();
    assertTrue(io.attemptStrategy(PoseSolveStrategy.CONSTRAINED_SOLVEPNP, result).isEmpty());
    heading.heading = Optional.of(Rotation2d.fromDegrees(15.0));
    assertTrue(
        io(new FakeCameraSource(List.of()), estimator, heading)
            .attemptStrategy(PoseSolveStrategy.CONSTRAINED_SOLVEPNP, result)
            .isEmpty());
    FakeCameraSource matrixOnly = new FakeCameraSource(List.of());
    matrixOnly.cameraMatrix = Optional.of(cameraMatrix());
    assertTrue(
        io(matrixOnly, estimator, heading)
            .attemptStrategy(PoseSolveStrategy.CONSTRAINED_SOLVEPNP, result)
            .isEmpty());
    FakeCameraSource distortionOnly = new FakeCameraSource(List.of());
    distortionOnly.distortion = Optional.of(distortion());
    assertTrue(
        io(distortionOnly, estimator, heading)
            .attemptStrategy(PoseSolveStrategy.CONSTRAINED_SOLVEPNP, result)
            .isEmpty());
    assertTrue(
        io(new FakeCameraSource(List.of()), estimator, null)
            .attemptStrategy(PoseSolveStrategy.CONSTRAINED_SOLVEPNP, result)
            .isEmpty());
  }

  @ParameterizedTest(name = "{0}: {1}")
  @MethodSource("invalidHistoryTimestampCases")
  void invalidRawTimestampsSkipHistoryStrategiesButPreserveNonHistoryFallback(
      String name, PoseSolveStrategy historyStrategy, double timestampSeconds) {
    PhotonTrackedTarget target = target(18, 1.0, 0.1, 0.0, 0.0);
    PhotonPipelineResult result = timestampedResult(timestampSeconds, target);
    FakeCameraSource source = new FakeCameraSource(List.of(List.of(result)));
    source.cameraMatrix = Optional.of(cameraMatrix());
    source.distortion = Optional.of(distortion());
    RecordingEstimator estimator = new RecordingEstimator();
    EstimatedRobotPose estimate = estimate(result, Pose3d.kZero, target);
    estimator.lowestResult = Optional.of(estimate);
    estimator.trigResult = Optional.of(estimate);
    estimator.constrainedResult = Optional.of(estimate);
    ProbeHeadingProvider heading = new ProbeHeadingProvider(0.0, 0.0);
    VisionIOPhotonVision io =
        new VisionIOPhotonVision(
            CAMERA,
            config(historyStrategy, PoseSolveStrategy.LOWEST_AMBIGUITY),
            source,
            estimator,
            heading);
    io.markVisionInitializationComplete();
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    io.updateInputs(inputs, NOW_SECONDS);

    PoseObservation[] observations = inputs.getPoseObservations();
    assertAll(
        () -> assertTrue(heading.headingTimestamps.isEmpty()),
        () -> assertTrue(heading.seedTimestamps.isEmpty()),
        () -> assertTrue(estimator.headingDataTimestamps.isEmpty()),
        () -> assertEquals(List.of("lowest"), estimator.events),
        () -> assertEquals(1, observations.length),
        () -> assertEquals(PoseSolveStrategy.LOWEST_AMBIGUITY, observations[0].strategy()),
        () ->
            assertEquals(
                Double.doubleToRawLongBits(timestampSeconds),
                Double.doubleToRawLongBits(observations[0].timestampSeconds())));
  }

  @ParameterizedTest(name = "{0}: {1}")
  @MethodSource("historyTimestampBoundaryCases")
  void inclusiveRawTimestampBoundariesAllowHistoryStrategies(
      String name, PoseSolveStrategy historyStrategy, double timestampSeconds) {
    PhotonTrackedTarget target = target(19, 1.0, 0.1, 0.0, 0.0);
    PhotonPipelineResult result = timestampedResult(timestampSeconds, target);
    FakeCameraSource source = new FakeCameraSource(List.of(List.of(result)));
    source.cameraMatrix = Optional.of(cameraMatrix());
    source.distortion = Optional.of(distortion());
    RecordingEstimator estimator = new RecordingEstimator();
    EstimatedRobotPose estimate = estimate(result, Pose3d.kZero, target);
    estimator.trigResult = Optional.of(estimate);
    estimator.constrainedResult = Optional.of(estimate);
    ProbeHeadingProvider heading = new ProbeHeadingProvider(0.0, 0.0);
    VisionIOPhotonVision io =
        new VisionIOPhotonVision(CAMERA, config(historyStrategy), source, estimator, heading);
    io.markVisionInitializationComplete();
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    io.updateInputs(inputs, NOW_SECONDS);

    PoseObservation[] observations = inputs.getPoseObservations();
    assertAll(
        () -> assertEquals(List.of(timestampSeconds), heading.headingTimestamps),
        () -> assertEquals(List.of(timestampSeconds), estimator.headingDataTimestamps),
        () ->
            assertEquals(
                historyStrategy == PoseSolveStrategy.CONSTRAINED_SOLVEPNP
                    ? List.of(timestampSeconds)
                    : List.of(),
                heading.seedTimestamps),
        () -> assertEquals(1, observations.length),
        () -> assertEquals(historyStrategy, observations[0].strategy()));
  }

  @Test
  void invalidRawFrameClearsPriorObservationWithoutTouchingHistoryState() {
    PhotonTrackedTarget target = target(20, 1.0, 0.1, 0.0, 0.0);
    PhotonPipelineResult valid = timestampedResult(99.9, target);
    PhotonPipelineResult invalid = timestampedResult(Double.NaN, target);
    FakeCameraSource source = new FakeCameraSource(List.of(List.of(valid), List.of(invalid)));
    RecordingEstimator estimator = new RecordingEstimator();
    estimator.trigResult = Optional.of(estimate(valid, Pose3d.kZero, target));
    ProbeHeadingProvider heading = new ProbeHeadingProvider(0.0, 0.0);
    VisionIOPhotonVision io =
        new VisionIOPhotonVision(
            CAMERA, config(PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE), source, estimator, heading);
    io.markVisionInitializationComplete();
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    io.updateInputs(inputs, NOW_SECONDS);
    assertEquals(1, inputs.getPoseObservations().length);
    heading.headingTimestamps.clear();
    estimator.headingDataTimestamps.clear();
    estimator.headingData.clear();
    estimator.events.clear();
    estimator.trigResult = Optional.of(estimate(invalid, Pose3d.kZero, target));

    io.updateInputs(inputs, NOW_SECONDS);

    assertAll(
        () -> assertEquals(0, inputs.getPoseObservations().length),
        () -> assertArrayEquals(new int[0], inputs.getTagIds()),
        () -> assertTrue(heading.headingTimestamps.isEmpty()),
        () -> assertTrue(heading.seedTimestamps.isEmpty()),
        () -> assertTrue(estimator.headingDataTimestamps.isEmpty()),
        () -> assertTrue(estimator.events.isEmpty()));
  }

  private static Stream<Arguments> invalidHistoryTimestampCases() {
    return Stream.of(
            Double.NaN,
            Double.POSITIVE_INFINITY,
            Double.NEGATIVE_INFINITY,
            NOW_SECONDS + VisionConstants.MAX_FUTURE_TIMESTAMP_SECONDS + 1e-9,
            NOW_SECONDS - VisionConstants.MAX_OBSERVATION_AGE_SECONDS - 1e-9)
        .flatMap(
            timestampSeconds ->
                Stream.of(
                    Arguments.of(
                        timestampCaseName(timestampSeconds),
                        PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
                        timestampSeconds),
                    Arguments.of(
                        timestampCaseName(timestampSeconds),
                        PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
                        timestampSeconds)));
  }

  private static Stream<Arguments> historyTimestampBoundaryCases() {
    return Stream.of(
        Arguments.of(
            "future boundary",
            PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
            NOW_SECONDS + VisionConstants.MAX_FUTURE_TIMESTAMP_SECONDS),
        Arguments.of(
            "future boundary",
            PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
            NOW_SECONDS + VisionConstants.MAX_FUTURE_TIMESTAMP_SECONDS),
        Arguments.of(
            "age boundary",
            PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
            NOW_SECONDS - VisionConstants.MAX_OBSERVATION_AGE_SECONDS),
        Arguments.of(
            "age boundary",
            PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
            NOW_SECONDS - VisionConstants.MAX_OBSERVATION_AGE_SECONDS));
  }

  private static String timestampCaseName(double timestampSeconds) {
    if (Double.isNaN(timestampSeconds)) {
      return "NaN";
    }
    if (timestampSeconds == Double.POSITIVE_INFINITY) {
      return "positive infinity";
    }
    if (timestampSeconds == Double.NEGATIVE_INFINITY) {
      return "negative infinity";
    }
    return timestampSeconds > NOW_SECONDS ? "future" : "stale";
  }

  private static PoseObservation observe(
      PhotonPipelineResult result, TagDistanceConfidenceMode distanceMode) {
    FakeCameraSource source = new FakeCameraSource(List.of(List.of(result)));
    ScriptedVisionIO io = scriptedIo(source, config(distanceMode));
    io.script(
        result,
        PoseSolveStrategy.LOWEST_AMBIGUITY,
        estimate(result, Pose3d.kZero, result.getTargets().toArray(PhotonTrackedTarget[]::new)));
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    io.updateInputs(inputs, result.getTimestampSeconds());
    return inputs.getPoseObservations()[0];
  }

  private static ScriptedVisionIO scriptedIo(FakeCameraSource source, VisionRuntimeConfig config) {
    return new ScriptedVisionIO(source, config, new ProbeHeadingProvider(0.0, 0.0));
  }

  private static VisionIOPhotonVision io(
      FakeCameraSource source,
      PhotonPoseEstimator estimator,
      VisionIOPhotonVision.HeadingProvider heading) {
    return new VisionIOPhotonVision(
        CAMERA, config(TagDistanceConfidenceMode.ALL_TAG_AVERAGE), source, estimator, heading);
  }

  private static VisionRuntimeConfig config(TagDistanceConfidenceMode distanceMode) {
    return new VisionRuntimeConfig(
        8.0,
        true,
        "STATIC",
        List.of(),
        false,
        distanceMode,
        StartupStrategyOrder.CONSTRAINED_SECOND);
  }

  private static VisionRuntimeConfig config(PoseSolveStrategy... strategyOrder) {
    return new VisionRuntimeConfig(
        8.0,
        true,
        "STATIC",
        List.of(strategyOrder),
        true,
        TagDistanceConfidenceMode.ALL_TAG_AVERAGE,
        StartupStrategyOrder.CONSTRAINED_SECOND);
  }

  private static EstimatedRobotPose estimate(
      PhotonPipelineResult result, Pose3d pose, PhotonTrackedTarget... targets) {
    return new EstimatedRobotPose(pose, result.getTimestampSeconds(), List.of(targets));
  }

  private static PhotonPipelineResult result(
      double timestampSeconds, PhotonTrackedTarget... targets) {
    long captureMicros = Math.round(timestampSeconds * 1_000_000.0);
    return new PhotonPipelineResult(
        captureMicros, captureMicros, captureMicros + 10_000, 0, List.of(targets));
  }

  private static PhotonPipelineResult timestampedResult(
      double timestampSeconds, PhotonTrackedTarget... targets) {
    return new TimestampedResult(timestampSeconds, targets);
  }

  private static PhotonTrackedTarget target(
      int fiducialId, Double distanceMeters, double ambiguity, double yaw, double pitch) {
    Transform3d transform =
        distanceMeters == null
            ? null
            : new Transform3d(new Translation3d(distanceMeters, 0.0, 0.0), Rotation3d.kZero);
    List<TargetCorner> corners =
        List.of(
            new TargetCorner(0.0, 0.0),
            new TargetCorner(1.0, 0.0),
            new TargetCorner(1.0, 1.0),
            new TargetCorner(0.0, 1.0));
    return new PhotonTrackedTarget(
        yaw,
        pitch,
        1.0,
        0.0,
        fiducialId,
        -1,
        -1.0f,
        transform,
        transform,
        ambiguity,
        corners,
        corners);
  }

  private static Matrix<N3, N3> cameraMatrix() {
    return MatBuilder.fill(Nat.N3(), Nat.N3(), 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
  }

  private static Matrix<N8, N1> distortion() {
    return VecBuilder.fill(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  private static final class ScriptedVisionIO extends VisionIOPhotonVision {
    private final Map<PhotonPipelineResult, Map<PoseSolveStrategy, EstimatedRobotPose>> scripts =
        new IdentityHashMap<>();
    private final List<PoseSolveStrategy> attempts = new ArrayList<>();

    ScriptedVisionIO(
        FakeCameraSource source, VisionRuntimeConfig config, HeadingProvider headingProvider) {
      super(
          CAMERA,
          config,
          source,
          new PhotonPoseEstimator(FieldConstants.APTAG_FIELD_LAYOUT, CAMERA.robotToCamera()),
          headingProvider);
    }

    void script(
        PhotonPipelineResult result, PoseSolveStrategy strategy, EstimatedRobotPose estimate) {
      scripts
          .computeIfAbsent(result, ignored -> new java.util.EnumMap<>(PoseSolveStrategy.class))
          .put(strategy, estimate);
    }

    List<PoseSolveStrategy> attempts() {
      return List.copyOf(attempts);
    }

    @Override
    Optional<EstimatedRobotPose> attemptStrategy(
        PoseSolveStrategy strategy, PhotonPipelineResult result) {
      attempts.add(strategy);
      return Optional.ofNullable(scripts.getOrDefault(result, Map.of()).get(strategy));
    }
  }

  private static final class TimestampedResult extends PhotonPipelineResult {
    private final double timestampSeconds;

    TimestampedResult(double timestampSeconds, PhotonTrackedTarget... targets) {
      super(0, 0, 10_000, 0, List.of(targets));
      this.timestampSeconds = timestampSeconds;
    }

    @Override
    public double getTimestampSeconds() {
      return timestampSeconds;
    }
  }

  private static final class FakeCameraSource implements VisionIOPhotonVision.CameraSource {
    private final Deque<List<PhotonPipelineResult>> batches;
    boolean connected = true;
    int drainCalls;
    Optional<Matrix<N3, N3>> cameraMatrix = Optional.empty();
    Optional<Matrix<N8, N1>> distortion = Optional.empty();

    FakeCameraSource(List<List<PhotonPipelineResult>> batches) {
      this.batches = new ArrayDeque<>(batches);
    }

    @Override
    public String name() {
      return CAMERA.name();
    }

    @Override
    public boolean isConnected() {
      return connected;
    }

    @Override
    public List<PhotonPipelineResult> getAllUnreadResults() {
      drainCalls++;
      return batches.isEmpty() ? List.of() : batches.removeFirst();
    }

    @Override
    public Optional<Matrix<N3, N3>> cameraMatrix() {
      return cameraMatrix;
    }

    @Override
    public Optional<Matrix<N8, N1>> distortionCoefficients() {
      return distortion;
    }
  }

  private static final class ProbeHeadingProvider implements VisionIOPhotonVision.HeadingProvider {
    double linearSpeed;
    double angularRate;
    Optional<Rotation2d> heading = Optional.of(Rotation2d.kZero);
    Optional<Pose3d> seed = Optional.of(Pose3d.kZero);
    final List<Double> headingTimestamps = new ArrayList<>();
    final List<Double> seedTimestamps = new ArrayList<>();

    ProbeHeadingProvider(double linearSpeed, double angularRate) {
      this.linearSpeed = linearSpeed;
      this.angularRate = angularRate;
    }

    @Override
    public Optional<Rotation2d> headingAt(double fpgaTimestampSeconds) {
      headingTimestamps.add(fpgaTimestampSeconds);
      return heading;
    }

    @Override
    public Optional<Pose3d> seedPoseAt(double fpgaTimestampSeconds) {
      seedTimestamps.add(fpgaTimestampSeconds);
      return seed;
    }

    @Override
    public double angularRateRadPerSecond() {
      return angularRate;
    }

    @Override
    public double linearSpeedMetersPerSecond() {
      return linearSpeed;
    }
  }

  private static final class RecordingEstimator extends PhotonPoseEstimator {
    final List<String> events = new ArrayList<>();
    final List<Double> headingDataTimestamps = new ArrayList<>();
    final List<Rotation2d> headingData = new ArrayList<>();
    Optional<EstimatedRobotPose> multiResult = Optional.empty();
    Optional<EstimatedRobotPose> lowestResult = Optional.empty();
    Optional<EstimatedRobotPose> trigResult = Optional.empty();
    Optional<EstimatedRobotPose> constrainedResult = Optional.empty();
    Pose3d constrainedSeed;
    boolean headingFree;
    double headingScale;

    RecordingEstimator() {
      super(FieldConstants.APTAG_FIELD_LAYOUT, CAMERA.robotToCamera());
    }

    @Override
    public void addHeadingData(double timestampSeconds, Rotation2d heading) {
      events.add("heading");
      headingDataTimestamps.add(timestampSeconds);
      headingData.add(heading);
      super.addHeadingData(timestampSeconds, heading);
    }

    @Override
    public Optional<EstimatedRobotPose> estimateCoprocMultiTagPose(PhotonPipelineResult result) {
      events.add("multi");
      return multiResult;
    }

    @Override
    public Optional<EstimatedRobotPose> estimateLowestAmbiguityPose(PhotonPipelineResult result) {
      events.add("lowest");
      return lowestResult;
    }

    @Override
    public Optional<EstimatedRobotPose> estimatePnpDistanceTrigSolvePose(
        PhotonPipelineResult result) {
      events.add("trig");
      return trigResult;
    }

    @Override
    public Optional<EstimatedRobotPose> estimateConstrainedSolvepnpPose(
        PhotonPipelineResult result,
        Matrix<N3, N3> cameraMatrix,
        Matrix<N8, N1> distortion,
        Pose3d seedPose,
        boolean headingFree,
        double headingScaleFactor) {
      events.add("constrained");
      constrainedSeed = seedPose;
      this.headingFree = headingFree;
      headingScale = headingScaleFactor;
      return constrainedResult;
    }
  }
}
