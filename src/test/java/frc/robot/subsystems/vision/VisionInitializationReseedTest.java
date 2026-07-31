package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.vision.Vision.AcceptedObservationSnapshot;
import frc.robot.subsystems.vision.Vision.InitializationState;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.PoseSolveStrategy;
import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import frc.robot.subsystems.vision.VisionRuntimeConfig.TagDistanceConfidenceMode;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogReplaySource;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

class VisionInitializationReseedTest {
  private static final VisionRuntimeConfig CONFIG =
      new VisionRuntimeConfig(
          8.0,
          false,
          "HYBRID",
          List.of(),
          false,
          TagDistanceConfidenceMode.ALL_TAG_AVERAGE,
          StartupStrategyOrder.CONSTRAINED_SECOND);

  @Test
  void initializationStateUsesInclusiveMotionBoundariesStrictTimeAndWrappedHeading() {
    InitializationState state = new InitializationState();
    InitializationState restarted = new InitializationState();
    InitializationState wrapped = new InitializationState();

    assertAll(
        () -> assertEquals(1, state.observe(pose(1.0, 1.0, 0.0), 1.0)),
        () -> assertEquals(2, state.observe(pose(1.20, 1.0, 10.0), 1.1)),
        () -> assertEquals(1, state.observe(pose(1.400000001, 1.0, 10.0), 1.2)),
        () -> assertEquals(1, state.observe(pose(1.400000001, 1.0, 10.0), 1.2)),
        () -> assertEquals(1, state.observe(pose(1.400000001, 1.0, 20.000001), 1.3)),
        () -> assertEquals(1, restarted.observe(pose(3.0, 1.0, 0.0), 3.0)),
        () -> assertEquals(1, restarted.observe(pose(3.200001, 1.0, 0.0), 3.1)),
        () -> assertEquals(2, restarted.observe(pose(3.390001, 1.0, 0.0), 3.2)),
        () -> assertEquals(1, wrapped.observe(pose(2.0, 2.0, 179.0), 2.0)),
        () -> assertEquals(2, wrapped.observe(pose(2.0, 2.0, -179.0), 2.1)));
  }

  @Test
  void onlyConsensusWinnerChangesItsFixedCameraInitializationAndSnapshotState() {
    RecordingDrive drive = new RecordingDrive();
    drive.currentPose = pose(2.0, 2.0, 0.0);
    MutableEnvironment environment = new MutableEnvironment(false, 50.0);
    SequenceIO winner =
        new SequenceIO("fixed-front", coprocessor(49.98, 2.0, 2.0, 0.0, 2, new int[] {1, 2}));
    SequenceIO clusterMate =
        new SequenceIO("cluster-mate", photon(49.99, 2.1, 2.0, 0.0, 2, new int[] {3, 4}));
    SequenceIO spatialLoser =
        new SequenceIO("spatial-loser", coprocessor(50.0, 7.0, 2.0, 0.0, 2, new int[] {5, 6}));
    SequenceIO temporalLoser =
        new SequenceIO("temporal-loser", coprocessor(49.80, 6.0, 2.0, 0.0, 2, new int[] {7, 8}));
    Vision vision =
        vision(environment, drive, true, winner, clusterMate, spatialLoser, temporalLoser);

    vision.periodic();

    AcceptedObservationSnapshot snapshot =
        vision.getLatestAcceptedObservationSnapshot().orElseThrow();
    assertAll(
        () -> assertEquals(1, drive.measurements.size()),
        () -> assertEquals(pose(2.0, 2.0, 0.0), drive.measurements.get(0)),
        () ->
            assertEquals(
                1, vision.initializationState("fixed-front").orElseThrow().stablePoseCount()),
        () -> assertTrue(vision.initializationState("cluster-mate").isEmpty()),
        () -> assertTrue(vision.initializationState("spatial-loser").isEmpty()),
        () -> assertTrue(vision.initializationState("temporal-loser").isEmpty()),
        () -> assertEquals(pose(2.0, 2.0, 0.0), snapshot.pose()),
        () -> assertArrayEquals(new int[] {1, 2}, snapshot.tagIds()),
        () -> assertEquals(49.98, snapshot.timestampSeconds(), 0.0));
  }

  @Test
  void initializationUsesTagCountRatherThanObservationIdArrayLength() {
    RecordingDrive drive = new RecordingDrive();
    drive.currentPose = pose(2.0, 2.0, 0.0);
    MutableEnvironment environment = new MutableEnvironment(false, 20.0);
    SequenceIO camera =
        new SequenceIO("front", coprocessor(19.90, 2.0, 2.0, 0.0, 1, new int[] {1, 2}))
            .then(coprocessor(19.95, 2.0, 2.0, 0.0, 2, new int[] {1}));
    Vision vision = vision(environment, drive, true, camera);

    vision.periodic();
    assertTrue(vision.initializationState("front").isEmpty());

    environment.now = 20.1;
    vision.periodic();

    assertAll(
        () -> assertEquals(1, vision.initializationState("front").orElseThrow().stablePoseCount()),
        () ->
            assertArrayEquals(
                new int[] {1},
                vision.getLatestAcceptedObservationSnapshot().orElseThrow().tagIds()));
  }

  @Test
  void selectedOtherCameraAndSolverDoNotEraseAnExistingCameraStreak() {
    RecordingDrive drive = new RecordingDrive();
    drive.currentPose = pose(2.0, 2.0, 0.0);
    MutableEnvironment environment = new MutableEnvironment(false, 25.0);
    SequenceIO front =
        new SequenceIO("front", coprocessor(24.90, 2.0, 2.0, 0.0, 2, new int[] {1, 2}))
            .then()
            .then(coprocessor(25.10, 2.1, 2.0, 0.0, 2, new int[] {1, 2}));
    SequenceIO rear =
        new SequenceIO("rear").then(photon(25.00, 2.0, 2.0, 0.0, 2, new int[] {3, 4})).then();
    Vision vision = vision(environment, drive, true, front, rear);

    vision.periodic();
    environment.now = 25.1;
    vision.periodic();
    environment.now = 25.2;
    vision.periodic();

    assertAll(
        () -> assertEquals(2, vision.initializationState("front").orElseThrow().stablePoseCount()),
        () -> assertTrue(vision.initializationState("rear").isEmpty()));
  }

  @Test
  void selectedAlternateSolverOnTheSameCameraPreservesThatCamerasStreak() {
    RecordingDrive drive = new RecordingDrive();
    drive.currentPose = pose(2.0, 2.0, 0.0);
    MutableEnvironment environment = new MutableEnvironment(false, 27.0);
    SequenceIO front =
        new SequenceIO("front", coprocessor(26.90, 2.0, 2.0, 0.0, 2, new int[] {1, 2}))
            .then(photon(27.00, 2.05, 2.0, 0.0, 2, new int[] {1, 2}))
            .then(coprocessor(27.10, 2.1, 2.0, 0.0, 2, new int[] {1, 2}));
    Vision vision = vision(environment, drive, true, front);

    vision.periodic();
    assertEquals(1, vision.initializationState("front").orElseThrow().stablePoseCount());

    environment.now = 27.1;
    vision.periodic();
    assertAll(
        () -> assertEquals(2, drive.measurements.size()),
        () -> assertEquals(1, vision.initializationState("front").orElseThrow().stablePoseCount()));

    environment.now = 27.2;
    vision.periodic();

    assertAll(
        () -> assertEquals(3, drive.measurements.size()),
        () -> assertEquals(2, vision.initializationState("front").orElseThrow().stablePoseCount()));
  }

  @Test
  void fiveStableSelectedObservationsReleaseEveryIoExactlyOnce() {
    RecordingDrive drive = new RecordingDrive();
    drive.currentPose = pose(2.0, 2.0, 0.0);
    MutableEnvironment environment = new MutableEnvironment(false, 30.0);
    SequenceIO front =
        new SequenceIO("front", coprocessor(29.80, 2.00, 2.0, 0.0, 2, new int[] {1, 2}))
            .then(coprocessor(29.85, 2.05, 2.0, 2.0, 2, new int[] {1, 2}))
            .then(coprocessor(29.90, 2.10, 2.0, 4.0, 2, new int[] {1, 2}))
            .then(coprocessor(29.95, 2.15, 2.0, 6.0, 2, new int[] {1, 2}))
            .then(coprocessor(30.00, 2.20, 2.0, 8.0, 2, new int[] {1, 2}))
            .then(coprocessor(30.05, 2.25, 2.0, 10.0, 2, new int[] {1, 2}));
    SequenceIO rear = new SequenceIO("rear", new PoseObservation[0]);
    Vision vision = vision(environment, drive, true, front, rear);

    for (int loop = 0; loop < 6; loop++) {
      environment.now = 30.0 + loop * 0.05;
      vision.periodic();
    }

    assertAll(
        () -> assertEquals(6, drive.measurements.size()),
        () -> assertEquals(1, front.initializationCompleteCalls),
        () -> assertEquals(1, rear.initializationCompleteCalls),
        () -> assertTrue(vision.isInitializationComplete()),
        () -> assertEquals(5, vision.initializationState("front").orElseThrow().stablePoseCount()));
  }

  @Test
  void releaseDisabledPreservesCompletionStreakSnapshotFusionAndLogging() throws Exception {
    RecordingDrive drive = new RecordingDrive();
    drive.currentPose = pose(2.0, 2.0, 0.0);
    MutableEnvironment environment = new MutableEnvironment(false, 60.0);
    SequenceIO front =
        new SequenceIO("front", coprocessor(59.80, 2.00, 2.0, 0.0, 2, new int[] {1, 2}))
            .then(coprocessor(59.85, 2.05, 2.0, 0.0, 2, new int[] {1, 2}))
            .then(coprocessor(59.90, 2.10, 2.0, 0.0, 2, new int[] {1, 2}))
            .then(coprocessor(59.95, 2.15, 2.0, 0.0, 2, new int[] {1, 2}))
            .then(coprocessor(60.00, 2.20, 2.0, 0.0, 2, new int[] {1, 2}));

    try (LoggerHarness logger = new LoggerHarness()) {
      Vision vision = vision(environment, drive, false, front);
      for (int loop = 0; loop < 5; loop++) {
        environment.now = 60.0 + loop * 0.05;
        vision.periodic();
      }

      assertAll(
          () -> assertEquals(5, drive.measurements.size()),
          () -> assertEquals(0, front.initializationCompleteCalls),
          () -> assertTrue(vision.isInitializationComplete()),
          () ->
              assertEquals(5, vision.initializationState("front").orElseThrow().stablePoseCount()),
          () ->
              assertEquals(
                  60.00,
                  vision.getLatestAcceptedObservationSnapshot().orElseThrow().timestampSeconds(),
                  0.0),
          () -> assertTrue(logger.bool("Vision/Initialization/Complete")),
          () -> assertEquals(5, logger.integer("Vision/Initialization/front/StablePoseCount")));
    }
  }

  @Test
  void snapshotReplacementFreshnessAndCopiesAreStrict() {
    RecordingDrive drive = new RecordingDrive();
    drive.currentPose = pose(2.0, 1.0, 0.0);
    MutableEnvironment environment = new MutableEnvironment(false, 70.1);
    int[] sourceIds = {1, 2};
    SequenceIO camera =
        new SequenceIO("front", photon(70.0, 1.0, 1.0, 0.0, 2, sourceIds))
            .then(photon(70.0, 2.0, 1.0, 0.0, 2, new int[] {3, 4}))
            .then(photon(69.99, 3.0, 1.0, 0.0, 2, new int[] {5, 6}));
    sourceIds[0] = 99;
    Vision vision = vision(environment, drive, true, camera);

    vision.periodic();
    vision.periodic();
    vision.periodic();
    int[] returnedIds = vision.getLatestAcceptedObservationSnapshot().orElseThrow().tagIds();
    returnedIds[0] = 88;

    int[] directSourceIds = {9, 10};
    AcceptedObservationSnapshot directSnapshot =
        new AcceptedObservationSnapshot(pose(4.0, 4.0, 0.0), directSourceIds, 5.0);
    directSourceIds[0] = 77;
    int[] directReturnedIds = directSnapshot.tagIds();
    directReturnedIds[1] = 66;

    environment.now = 70.5;
    AcceptedObservationSnapshot boundary =
        vision.getLatestAcceptedObservationSnapshot().orElseThrow();
    environment.now = 70.500000001;

    assertAll(
        () -> assertEquals(pose(1.0, 1.0, 0.0), boundary.pose()),
        () -> assertArrayEquals(new int[] {1, 2}, boundary.tagIds()),
        () -> assertEquals(70.0, boundary.timestampSeconds(), 0.0),
        () -> assertArrayEquals(new int[] {9, 10}, directSnapshot.tagIds()),
        () -> assertTrue(vision.getLatestAcceptedObservationSnapshot().isEmpty()));
  }

  @Test
  void disabledNonCoprocessorCannotEraseRecentEnabledSnapshotButNewCoprocessorCanReplaceIt() {
    RecordingDrive drive = new RecordingDrive();
    MutableEnvironment environment = new MutableEnvironment(false, 80.1);
    SequenceIO camera =
        new SequenceIO("front", photon(80.0, 1.0, 1.0, 0.0, 2, new int[] {1, 2}))
            .then(photon(80.1, 3.0, 1.0, 0.0, 2, new int[] {3, 4}))
            .then(coprocessor(80.2, 2.0, 1.0, 0.0, 2, new int[] {5, 6}));
    Vision vision = vision(environment, drive, true, camera);

    vision.periodic();
    assertEquals(0, drive.estimatorResets.size());
    environment.disabled = true;
    environment.now = 80.2;
    vision.periodic();
    Pose2d initialDisabledReset = drive.estimatorResets.get(0);
    environment.now = 80.3;
    vision.periodic();

    assertAll(
        () -> assertEquals(pose(1.0, 1.0, 0.0), initialDisabledReset),
        () -> assertEquals(1, drive.estimatorResets.size()),
        () ->
            assertEquals(
                pose(2.0, 1.0, 0.0),
                vision.getLatestAcceptedObservationSnapshot().orElseThrow().pose()),
        () ->
            assertArrayEquals(
                new int[] {5, 6},
                vision.getLatestAcceptedObservationSnapshot().orElseThrow().tagIds()));
  }

  @Test
  void automaticReseedEnforcesDriftCooldownAndDisabledCycleBoundaries() {
    RecordingDrive drive = new RecordingDrive();
    MutableEnvironment environment = new MutableEnvironment(false, 9.95);
    SequenceIO camera =
        new SequenceIO("front", photon(9.90, 1.0, 1.0, 0.0, 2, new int[] {1, 2}))
            .then()
            .then(coprocessor(10.48, 1.250001, 1.0, 0.0, 2, new int[] {1, 2}))
            .then(coprocessor(10.49, 1.25, 1.0, 0.0, 2, new int[] {1, 2}))
            .then(coprocessor(10.495, 1.250001, 1.0, 0.0, 2, new int[] {1, 2}))
            .then(photon(10.59, 1.250001, 1.0, 0.0, 2, new int[] {1, 2}))
            .then()
            .then();
    Vision vision = vision(environment, drive, true, camera);

    vision.periodic();
    environment.disabled = true;
    environment.now = 10.0;
    vision.periodic();
    assertEquals(1, drive.estimatorResets.size());

    environment.now = 10.499;
    vision.periodic();
    assertEquals(1, drive.estimatorResets.size());

    environment.now = 10.5;
    vision.periodic();
    assertEquals(1, drive.estimatorResets.size());

    vision.periodic();
    assertEquals(2, drive.estimatorResets.size());

    environment.disabled = false;
    environment.now = 10.6;
    vision.periodic();
    environment.disabled = true;
    environment.now = 10.9;
    vision.periodic();
    assertEquals(2, drive.estimatorResets.size());

    environment.now = 11.0;
    vision.periodic();

    assertAll(
        () -> assertEquals(3, drive.estimatorResets.size()),
        () -> assertEquals(pose(1.250001, 1.0, 0.0), drive.estimatorResets.get(2)),
        () -> assertEquals(1, camera.initializationCompleteCalls),
        () -> assertEquals(0, drive.truthResetCount));
  }

  @Test
  void automaticReseedRequiresFreshSnapshotWithTwoObservationIds() {
    RecordingDrive drive = new RecordingDrive();
    MutableEnvironment environment = new MutableEnvironment(true, 90.0);
    SequenceIO camera =
        new SequenceIO("front", coprocessor(89.9, 2.0, 2.0, 0.0, 2, new int[] {1})).then();
    Vision vision = vision(environment, drive, true, camera);

    vision.periodic();
    assertAll(
        () -> assertEquals(1, vision.initializationState("front").orElseThrow().stablePoseCount()),
        () -> assertEquals(0, drive.estimatorResets.size()));

    environment.now = 90.500000001;
    vision.periodic();
    assertEquals(0, drive.estimatorResets.size());
  }

  @Test
  void automaticReseedRejectsAStaleTwoIdSnapshot() {
    RecordingDrive drive = new RecordingDrive();
    MutableEnvironment environment = new MutableEnvironment(false, 95.0);
    SequenceIO camera =
        new SequenceIO("front", photon(94.9, 1.0, 1.0, 0.0, 2, new int[] {1, 2})).then();
    Vision vision = vision(environment, drive, true, camera);

    vision.periodic();
    environment.disabled = true;
    environment.now = 95.400000001;
    vision.periodic();

    assertEquals(0, drive.estimatorResets.size());
  }

  @Test
  void automaticReseedOrdersFusionAndCompletionBeforeEstimatorResetAndIsIdempotent() {
    List<String> events = new ArrayList<>();
    RecordingDrive drive = new RecordingDrive();
    drive.events = events;
    MutableEnvironment environment = new MutableEnvironment(true, 100.0);
    SequenceIO front =
        new SequenceIO("front", events, coprocessor(99.9, 2.0, 2.0, 0.0, 2, new int[] {1, 2}));
    SequenceIO rear = new SequenceIO("rear", events, new PoseObservation[0]);
    Vision vision = vision(environment, drive, true, front, rear);

    vision.periodic();

    assertAll(
        () ->
            assertEquals(
                List.of("measurement", "init-front", "init-rear", "estimator-reset"), events),
        () -> assertTrue(vision.isInitializationComplete()),
        () -> assertEquals(1, front.initializationCompleteCalls),
        () -> assertEquals(1, rear.initializationCompleteCalls),
        () -> assertEquals(1, drive.estimatorResets.size()),
        () -> assertEquals(0, drive.truthResetCount));

    environment.now = 100.5;
    drive.currentPose = Pose2d.kZero;
    vision.periodic();
    assertAll(
        () -> assertEquals(1, front.initializationCompleteCalls),
        () -> assertEquals(1, rear.initializationCompleteCalls));
  }

  @Test
  void automaticDriftDecisionUsesEstimatorPoseAfterSynchronousFusion() {
    RecordingDrive drive = new RecordingDrive();
    drive.fuseMeasurementsIntoCurrentPose = true;
    MutableEnvironment environment = new MutableEnvironment(true, 200.0);
    SequenceIO front =
        new SequenceIO("front", coprocessor(199.9, 1.0, 1.0, 0.0, 2, new int[] {1, 2}))
            .then(coprocessor(200.49, 2.0, 1.0, 0.0, 2, new int[] {1, 2}));
    Vision vision = vision(environment, drive, true, front);

    vision.periodic();
    assertEquals(1, drive.estimatorResets.size());

    drive.currentPose = Pose2d.kZero;
    environment.now = 200.5;
    vision.periodic();

    assertAll(
        () -> assertEquals(2, drive.measurements.size()),
        () -> assertEquals(pose(2.0, 1.0, 0.0), drive.currentPose),
        () -> assertEquals(1, drive.estimatorResets.size()));
  }

  @Test
  void
      manualReseedAcceptsFreshEnabledSingleTagButNotMissingOrStaleAndNeverCompletesInitialization() {
    RecordingDrive drive = new RecordingDrive();
    drive.currentPose = pose(2.0, 2.0, 0.0);
    MutableEnvironment environment = new MutableEnvironment(false, 110.0);
    SequenceIO camera =
        new SequenceIO("front", photon(109.9, 2.0, 2.0, 0.0, 1, new int[] {1})).then();
    Vision vision = vision(environment, drive, true, camera);

    assertFalse(vision.forceReseedFromVision());
    vision.periodic();
    assertTrue(vision.forceReseedFromVision());

    environment.now = 110.400000001;
    assertFalse(vision.forceReseedFromVision());

    assertAll(
        () -> assertEquals(List.of(pose(2.0, 2.0, 0.0)), drive.estimatorResets),
        () -> assertEquals(0, drive.truthResetCount),
        () -> assertFalse(vision.isInitializationComplete()),
        () -> assertEquals(0, camera.initializationCompleteCalls));
  }

  @Test
  void manualReseedSuccessLogIsWrittenImmediatelyAndClearedByTheNextLoop() throws Exception {
    RecordingDrive drive = new RecordingDrive();
    MutableEnvironment environment = new MutableEnvironment(false, 115.0);
    SequenceIO camera =
        new SequenceIO("front", photon(114.9, 1.0, 1.0, 0.0, 1, new int[] {1})).then();

    try (LoggerHarness logger = new LoggerHarness()) {
      Vision vision = vision(environment, drive, true, camera);
      vision.periodic();
      assertTrue(vision.forceReseedFromVision());
      assertTrue(logger.bool("Vision/Reseed/ManualSucceeded"));

      environment.now = 115.1;
      vision.periodic();

      assertAll(
          () -> assertFalse(logger.falseBool("Vision/Reseed/ManualSucceeded")),
          () -> assertFalse(logger.falseBool("Vision/Reseed/AutomaticSucceeded")),
          () -> assertEquals(Pose2d.kZero, logger.pose("Vision/Reseed/Pose")),
          () -> assertEquals(0.0, logger.number("Vision/Reseed/DeltaMeters"), 0.0),
          () -> assertEquals(0.0, logger.number("Vision/Reseed/TimestampSeconds"), 0.0),
          () -> assertEquals("", logger.string("Vision/Reseed/RejectionReason")));
    }
  }

  @Test
  void taskSevenLogsAreWrittenAndTransientReseedOutputsClearEveryLoop() throws Exception {
    RecordingDrive drive = new RecordingDrive();
    MutableEnvironment environment = new MutableEnvironment(true, 120.0);
    SequenceIO camera =
        new SequenceIO("front", coprocessor(119.9, 2.0, 2.0, 0.0, 2, new int[] {1, 2})).then();

    try (LoggerHarness logger = new LoggerHarness()) {
      Vision vision = vision(environment, drive, true, camera);
      vision.periodic();

      assertAll(
          () -> assertEquals(1, logger.integer("Vision/Initialization/front/StablePoseCount")),
          () -> assertTrue(logger.bool("Vision/Initialization/Complete")),
          () -> assertEquals(0.1, logger.number("Vision/Snapshot/AgeSeconds"), 1e-12),
          () -> assertArrayEquals(new int[] {1, 2}, logger.ints("Vision/Snapshot/TagIDs")),
          () -> assertTrue(logger.bool("Vision/Reseed/AutomaticSucceeded")),
          () -> assertFalse(logger.falseBool("Vision/Reseed/ManualSucceeded")),
          () -> assertEquals(pose(2.0, 2.0, 0.0), logger.pose("Vision/Reseed/Pose")),
          () -> assertEquals(Math.sqrt(8.0), logger.number("Vision/Reseed/DeltaMeters"), 1e-12),
          () -> assertEquals(119.9, logger.number("Vision/Reseed/TimestampSeconds"), 0.0),
          () -> assertEquals("", logger.string("Vision/Reseed/RejectionReason")));

      environment.disabled = false;
      environment.now = 120.1;
      vision.periodic();

      assertAll(
          () -> assertFalse(logger.falseBool("Vision/Reseed/AutomaticSucceeded")),
          () -> assertFalse(logger.falseBool("Vision/Reseed/ManualSucceeded")),
          () -> assertEquals(Pose2d.kZero, logger.pose("Vision/Reseed/Pose")),
          () -> assertEquals(0.0, logger.number("Vision/Reseed/DeltaMeters"), 0.0),
          () -> assertEquals(0.0, logger.number("Vision/Reseed/TimestampSeconds"), 0.0),
          () -> assertEquals("", logger.string("Vision/Reseed/RejectionReason")));
    }
  }

  private static Vision vision(
      MutableEnvironment environment,
      RecordingDrive drive,
      boolean releaseStartupStrategy,
      VisionIO... io) {
    return new Vision(
        drive.bindings(),
        CONFIG,
        () -> environment.disabled,
        () -> environment.now,
        () -> {},
        releaseStartupStrategy,
        io);
  }

  private static PoseObservation coprocessor(
      double timestamp, double x, double y, double headingDegrees, int tagCount, int[] tagIds) {
    return observation(
        timestamp,
        x,
        y,
        headingDegrees,
        tagCount,
        tagIds,
        PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
        PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
  }

  private static PoseObservation photon(
      double timestamp, double x, double y, double headingDegrees, int tagCount, int[] tagIds) {
    return observation(
        timestamp,
        x,
        y,
        headingDegrees,
        tagCount,
        tagIds,
        PoseObservationType.PHOTONVISION,
        PoseSolveStrategy.LOWEST_AMBIGUITY);
  }

  private static PoseObservation observation(
      double timestamp,
      double x,
      double y,
      double headingDegrees,
      int tagCount,
      int[] tagIds,
      PoseObservationType type,
      PoseSolveStrategy strategy) {
    return new PoseObservation(
        timestamp,
        new Pose3d(x, y, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(headingDegrees))),
        0.05,
        tagCount,
        2.0,
        type,
        strategy,
        tagIds);
  }

  private static Pose2d pose(double x, double y, double headingDegrees) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees));
  }

  private static final class MutableEnvironment {
    private boolean disabled;
    private double now;

    private MutableEnvironment(boolean disabled, double now) {
      this.disabled = disabled;
      this.now = now;
    }
  }

  private static final class SequenceIO implements VisionIO {
    private final String name;
    private final List<String> events;
    private final List<PoseObservation[]> frames = new ArrayList<>();
    private int frameIndex;
    private int initializationCompleteCalls;

    private SequenceIO(String name, PoseObservation... observations) {
      this(name, new ArrayList<>(), observations);
    }

    private SequenceIO(String name, List<String> events, PoseObservation... observations) {
      this.name = name;
      this.events = events;
      frames.add(observations);
    }

    private SequenceIO then(PoseObservation... observations) {
      frames.add(observations);
      return this;
    }

    @Override
    public String getCameraName() {
      return name;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      PoseObservation[] frame = frames.get(Math.min(frameIndex, frames.size() - 1));
      inputs.cameraName = "mutable-" + name;
      inputs.connected = true;
      inputs.setPoseObservations(frame);
      inputs.setTagIds(frame.length == 0 ? new int[0] : frame[frame.length - 1].tagIds());
      frameIndex++;
    }

    @Override
    public void markVisionInitializationComplete() {
      initializationCompleteCalls++;
      events.add("init-" + name);
    }
  }

  private static final class RecordingDrive {
    private final List<Pose2d> measurements = new ArrayList<>();
    private final List<Pose2d> estimatorResets = new ArrayList<>();
    private Pose2d currentPose = Pose2d.kZero;
    private int truthResetCount;
    private List<String> events;
    private boolean fuseMeasurementsIntoCurrentPose;

    private VisionDriveBindings bindings() {
      return new VisionDriveBindings(
          (pose, timestamp, stdDevs) -> {
            measurements.add(pose);
            if (fuseMeasurementsIntoCurrentPose) {
              currentPose = pose;
            }
            if (events != null) {
              events.add("measurement");
            }
          },
          pose -> {
            estimatorResets.add(pose);
            currentPose = pose;
            if (events != null) {
              events.add("estimator-reset");
            }
          },
          () -> currentPose,
          timestamp -> Optional.of(currentPose),
          ChassisSpeeds::new,
          () -> 0.0,
          () -> 0.0);
    }
  }

  private static final class LoggerHarness implements AutoCloseable {
    private final Field running = Logger.class.getDeclaredField("running");
    private final Field entryField = Logger.class.getDeclaredField("entry");
    private final Field outputTable = Logger.class.getDeclaredField("outputTable");
    private final Field replaySource = Logger.class.getDeclaredField("replaySource");
    private final boolean previousRunning;
    private final LogTable previousEntry;
    private final LogTable previousOutput;
    private final LogReplaySource previousReplay;
    private final LogTable entry = new LogTable(0);

    private LoggerHarness() throws ReflectiveOperationException {
      running.setAccessible(true);
      entryField.setAccessible(true);
      outputTable.setAccessible(true);
      replaySource.setAccessible(true);
      previousRunning = running.getBoolean(null);
      previousEntry = (LogTable) entryField.get(null);
      previousOutput = (LogTable) outputTable.get(null);
      previousReplay = (LogReplaySource) replaySource.get(null);
      running.setBoolean(null, true);
      entryField.set(null, entry);
      replaySource.set(null, null);
      outputTable.set(null, entry.getSubtable("RealOutputs"));
    }

    private Pose2d pose(String key) {
      return entry.get(
          "RealOutputs/" + key, new Pose2d(-999.0, -999.0, Rotation2d.fromDegrees(123.0)));
    }

    private int[] ints(String key) {
      return entry.get("RealOutputs/" + key, new int[0]);
    }

    private String string(String key) {
      return entry.get("RealOutputs/" + key, "__missing__");
    }

    private double number(String key) {
      return entry.get("RealOutputs/" + key, Double.NaN);
    }

    private long integer(String key) {
      return entry.get("RealOutputs/" + key, Long.MIN_VALUE);
    }

    private boolean bool(String key) {
      return entry.get("RealOutputs/" + key, false);
    }

    private boolean falseBool(String key) {
      return entry.get("RealOutputs/" + key, true);
    }

    @Override
    public void close() throws ReflectiveOperationException {
      replaySource.set(null, previousReplay);
      outputTable.set(null, previousOutput);
      entryField.set(null, previousEntry);
      running.setBoolean(null, previousRunning);
    }
  }
}
