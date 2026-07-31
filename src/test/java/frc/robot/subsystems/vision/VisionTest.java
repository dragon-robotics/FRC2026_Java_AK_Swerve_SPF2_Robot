package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.PoseSolveStrategy;
import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import frc.robot.subsystems.vision.VisionRuntimeConfig.TagDistanceConfidenceMode;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

class VisionTest {
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
  void periodicCapturesNowOnceAndRunsHookBeforeEveryCameraUpdate() {
    List<String> events = new ArrayList<>();
    AtomicInteger clockCalls = new AtomicInteger();
    ScriptedIO front = new ScriptedIO("front", events, true, observation(9.9, 1.0, 1.0));
    ScriptedIO rear = new ScriptedIO("rear", events, true, observation(9.9, 1.1, 1.0));
    RecordingDrive drive = new RecordingDrive();
    drive.events = events;

    Vision vision =
        new Vision(
            drive.bindings(),
            CONFIG,
            () -> false,
            () -> 10.0 + clockCalls.getAndIncrement(),
            () -> events.add("hook"),
            true,
            front,
            rear);

    vision.periodic();

    assertAll(
        () -> assertEquals(1, clockCalls.get()),
        () -> assertEquals(1, front.nameCalls),
        () -> assertEquals(1, rear.nameCalls),
        () ->
            assertEquals(
                List.of("hook", "update-front", "update-rear", "history", "history"), events),
        () -> assertEquals(2, drive.historyCalls),
        () -> assertEquals(1, drive.measurements.size()));
  }

  @Test
  void invalidTimestampsNeverRequestHistoryAndLogTheirExactReasons()
      throws ReflectiveOperationException {
    RecordingDrive drive = new RecordingDrive();
    drive.throwOnHistory = true;
    ScriptedIO nan = new ScriptedIO("nan", true, observation(Double.NaN, 1.0, 1.0));
    ScriptedIO infinity =
        new ScriptedIO("infinity", true, observation(Double.POSITIVE_INFINITY, 1.0, 1.0));
    ScriptedIO future = new ScriptedIO("future", true, observation(10.021, 1.0, 1.0));
    ScriptedIO stale = new ScriptedIO("stale", true, observation(9.499, 1.0, 1.0));

    try (LoggerHarness logger = new LoggerHarness(false)) {
      new Vision(
              drive.bindings(),
              CONFIG,
              () -> false,
              () -> 10.0,
              () -> {},
              true,
              nan,
              infinity,
              future,
              stale)
          .periodic();

      assertAll(
          () -> assertEquals(0, drive.historyCalls),
          () ->
              assertEquals("INVALID_TIMESTAMP", logger.outputString("Vision/nan/RejectionReason")),
          () ->
              assertEquals(
                  "INVALID_TIMESTAMP", logger.outputString("Vision/infinity/RejectionReason")),
          () ->
              assertEquals(
                  "FUTURE_TIMESTAMP", logger.outputString("Vision/future/RejectionReason")),
          () ->
              assertEquals("STALE_TIMESTAMP", logger.outputString("Vision/stale/RejectionReason")));
    }
  }

  @Test
  void validFrameWithMissingHistoryUsesCurrentPoseOnlyForInnovationAndPreservesMeasurement() {
    RecordingDrive drive = new RecordingDrive();
    drive.currentPose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(70.0));
    drive.timestampedPose = Optional.empty();
    PoseObservation frame = observation(4.25, 1.3, 1.4);
    Vision vision = vision(drive, () -> false, () -> 4.5, new ScriptedIO("front", true, frame));

    vision.periodic();

    Measurement accepted = drive.measurements.get(0);
    assertAll(
        () -> assertEquals(1, drive.historyCalls),
        () -> assertEquals(1, drive.currentPoseCalls),
        () -> assertEquals(frame.pose().toPose2d(), accepted.pose()),
        () -> assertEquals(4.25, accepted.timestampSeconds(), 0.0),
        () -> assertEquals(0.04, accepted.standardDeviations().get(0, 0), 1e-12),
        () -> assertEquals(0.04, accepted.standardDeviations().get(1, 0), 1e-12),
        () -> assertEquals(1e9, accepted.standardDeviations().get(2, 0), 0.0));
  }

  @Test
  void aimingDefaultsFalseAndAppliesExactTranslationUncertaintyFactor() {
    RecordingDrive drive = new RecordingDrive();
    ScriptedIO front =
        new ScriptedIO("front", true, observation(7.8, 1.0, 1.0), observation(7.9, 1.0, 1.0));
    Vision vision = vision(drive, () -> false, () -> 8.0, front);

    vision.periodic();
    vision.setAiming(true);
    vision.periodic();

    assertAll(
        () -> assertEquals(2, drive.measurements.size()),
        () -> assertEquals(0.04, drive.measurements.get(0).standardDeviations().get(0, 0), 1e-12),
        () -> assertEquals(0.024, drive.measurements.get(1).standardDeviations().get(0, 0), 1e-12));
  }

  @Test
  void fourCoherentCamerasProduceOneOriginalMeasurement() {
    RecordingDrive drive = new RecordingDrive();
    PoseObservation expected = observation(11.98, 2.08, 2.00);
    Vision vision =
        vision(
            drive,
            () -> true,
            () -> 12.0,
            new ScriptedIO("front", true, observation(11.97, 2.00, 2.00)),
            new ScriptedIO("right", true, expected),
            new ScriptedIO("rear", true, observation(11.99, 2.12, 2.02)),
            new ScriptedIO("left", true, observation(12.00, 2.15, 2.01)));

    vision.periodic();

    assertAll(
        () -> assertEquals(1, drive.measurements.size()),
        () -> assertEquals(expected.pose().toPose2d(), drive.measurements.get(0).pose()),
        () ->
            assertEquals(
                expected.timestampSeconds(), drive.measurements.get(0).timestampSeconds(), 0.0));
  }

  @Test
  void temporalWinnerIsChosenBeforeSpatialConsensus() throws ReflectiveOperationException {
    RecordingDrive drive = new RecordingDrive();
    PoseObservation newerTemporalWinner = observation(19.98, 7.0, 2.0);
    PoseObservation oldA = observation(19.80, 1.0, 1.0);
    PoseObservation oldB = observation(19.81, 1.01, 1.0);
    try (LoggerHarness logger = new LoggerHarness(false)) {
      Vision vision =
          vision(
              drive,
              () -> true,
              () -> 20.0,
              new ScriptedIO("old-a", true, oldA),
              new ScriptedIO("old-b", true, oldB),
              new ScriptedIO("new-a", true, newerTemporalWinner),
              new ScriptedIO("new-b", true, observation(19.99, 9.0, 2.0)));

      vision.periodic();

      assertAll(
          () -> assertEquals(1, drive.measurements.size()),
          () ->
              assertEquals(newerTemporalWinner.pose().toPose2d(), drive.measurements.get(0).pose()),
          () -> assertEquals(19.98, drive.measurements.get(0).timestampSeconds(), 0.0),
          () ->
              assertArrayEquals(
                  new Pose3d[] {oldA.pose()},
                  logger.outputPoses("Vision/old-a/SkewRejectedRobotPoses")),
          () ->
              assertArrayEquals(
                  new Pose3d[] {oldB.pose()},
                  logger.outputPoses("Vision/old-b/SkewRejectedRobotPoses")),
          () ->
              assertArrayEquals(
                  new Pose3d[0], logger.outputPoses("Vision/old-a/RejectedRobotPoses")),
          () ->
              assertEquals(
                  VisionConsensus.TIMESTAMP_CLUSTER_NOT_SELECTED,
                  logger.outputString("Vision/old-a/RejectionReason")));
    }
  }

  @Test
  void oneCameraBacklogContributesOnlyItsNewestVote() {
    RecordingDrive drive = new RecordingDrive();
    PoseObservation expected = observation(29.98, 3.0, 3.0);
    ScriptedIO backlog =
        new ScriptedIO(
            "backlog",
            true,
            observation(29.94, 8.0, 4.0),
            observation(29.96, 8.0, 4.0),
            observation(29.99, 8.0, 4.0));
    Vision vision =
        vision(
            drive,
            () -> true,
            () -> 30.0,
            backlog,
            new ScriptedIO("front", true, expected),
            new ScriptedIO("rear", true, observation(29.97, 3.1, 3.0)));

    vision.periodic();

    assertAll(
        () -> assertEquals(1, drive.measurements.size()),
        () -> assertEquals(expected.pose().toPose2d(), drive.measurements.get(0).pose()));
  }

  @Test
  void spatialLoserIsLoggedButNeverFusedAndNoCandidatesMakeNoCall()
      throws ReflectiveOperationException {
    RecordingDrive drive = new RecordingDrive();
    PoseObservation winner = observation(39.98, 2.0, 2.0);
    PoseObservation loser = observation(39.99, 7.0, 2.0);
    ScriptedIO front =
        new ScriptedIO("front", true, new PoseObservation[] {winner}, new PoseObservation[0]);
    ScriptedIO rear =
        new ScriptedIO("rear", true, new PoseObservation[] {loser}, new PoseObservation[0]);
    ScriptedIO left =
        new ScriptedIO(
            "left",
            true,
            new PoseObservation[] {observation(39.97, 2.1, 2.0)},
            new PoseObservation[0]);

    try (LoggerHarness logger = new LoggerHarness(false)) {
      Vision vision = vision(drive, () -> true, () -> 40.0, front, rear, left);
      vision.periodic();

      assertAll(
          () -> assertEquals(1, drive.measurements.size()),
          () -> assertEquals(winner.pose().toPose2d(), drive.measurements.get(0).pose()),
          () ->
              assertArrayEquals(
                  new Pose3d[] {loser.pose()},
                  logger.outputPoses("Vision/rear/RejectedRobotPoses")),
          () ->
              assertEquals(
                  "CONSENSUS_NOT_SELECTED", logger.outputString("Vision/rear/RejectionReason")));

      vision.periodic();
      assertEquals(1, drive.measurements.size());
    }
  }

  @Test
  void disconnectedCameraRaisesOnlyItsAlertAndHealthyCameraStillFuses()
      throws ReflectiveOperationException {
    RecordingDrive drive = new RecordingDrive();
    ScriptedIO disconnected = new ScriptedIO("disconnected", false);
    ScriptedIO healthy = new ScriptedIO("healthy", true, observation(49.9, 2.0, 2.0));
    Vision vision = vision(drive, () -> true, () -> 50.0, disconnected, healthy);

    vision.periodic();

    Field alertsField = Vision.class.getDeclaredField("disconnectedAlerts");
    alertsField.setAccessible(true);
    Alert[] alerts = (Alert[]) alertsField.get(vision);
    assertAll(
        () -> assertEquals(1, drive.measurements.size()),
        () -> assertTrue(alerts[0].get()),
        () -> assertFalse(alerts[1].get()),
        () -> assertTrue(alerts[0].getText().contains("disconnected")),
        () -> assertTrue(alerts[1].getText().contains("healthy")));
  }

  private static Vision vision(
      RecordingDrive drive,
      java.util.function.BooleanSupplier disabled,
      java.util.function.DoubleSupplier now,
      VisionIO... io) {
    return new Vision(drive.bindings(), CONFIG, disabled, now, () -> {}, true, io);
  }

  static PoseObservation observation(double timestamp, double x, double y) {
    return new PoseObservation(
        timestamp,
        new Pose3d(x, y, 0.0, new Rotation3d()),
        0.05,
        2,
        2.0,
        PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
        PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        new int[] {1, 2});
  }

  static final class ScriptedIO implements VisionIO {
    private final String name;
    private final List<String> events;
    private final boolean connected;
    private final List<PoseObservation[]> frames = new ArrayList<>();
    private int updateIndex;
    private int nameCalls;
    private int[] aggregateTagIds = {1, 2};

    ScriptedIO(String name, boolean connected, PoseObservation... observations) {
      this(name, new ArrayList<>(), connected, observations);
    }

    ScriptedIO(
        String name, List<String> events, boolean connected, PoseObservation... observations) {
      this.name = name;
      this.events = events;
      this.connected = connected;
      frames.add(observations);
    }

    ScriptedIO(String name, boolean connected, PoseObservation[] first, PoseObservation[] second) {
      this(name, connected, first);
      frames.add(second);
    }

    ScriptedIO withAggregateTagIds(int... ids) {
      aggregateTagIds = Arrays.copyOf(ids, ids.length);
      return this;
    }

    @Override
    public String getCameraName() {
      nameCalls++;
      return name;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      events.add("update-" + name);
      inputs.cameraName = "mutable-" + name;
      inputs.connected = connected;
      PoseObservation[] observations = frames.get(Math.min(updateIndex, frames.size() - 1));
      inputs.setPoseObservations(observations);
      inputs.setTagIds(observations.length == 0 ? new int[0] : aggregateTagIds);
      updateIndex++;
    }
  }

  static final class RecordingDrive {
    final List<Measurement> measurements = new ArrayList<>();
    Pose2d currentPose = Pose2d.kZero;
    Optional<Pose2d> timestampedPose = Optional.of(Pose2d.kZero);
    ChassisSpeeds speeds = new ChassisSpeeds();
    double pitch;
    double roll;
    int historyCalls;
    int currentPoseCalls;
    boolean throwOnHistory;
    List<String> events;

    VisionDriveBindings bindings() {
      return new VisionDriveBindings(
          (pose, timestamp, standardDeviations) ->
              measurements.add(new Measurement(pose, timestamp, standardDeviations)),
          pose -> currentPose = pose,
          () -> {
            currentPoseCalls++;
            return currentPose;
          },
          timestamp -> {
            historyCalls++;
            if (events != null) {
              events.add("history");
            }
            if (throwOnHistory) {
              throw new AssertionError("Invalid timestamp reached drive history: " + timestamp);
            }
            return timestampedPose;
          },
          () -> speeds,
          () -> pitch,
          () -> roll);
    }
  }

  record Measurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations) {}

  static final class LoggerHarness implements AutoCloseable {
    private final Field runningField = Logger.class.getDeclaredField("running");
    private final Field entryField = Logger.class.getDeclaredField("entry");
    private final Field outputTableField = Logger.class.getDeclaredField("outputTable");
    private final Field replaySourceField = Logger.class.getDeclaredField("replaySource");
    private final boolean previousRunning;
    private final LogTable previousEntry;
    private final LogTable previousOutput;
    private final Object previousReplaySource;
    final LogTable entry = new LogTable(0);

    LoggerHarness(boolean replay) throws ReflectiveOperationException {
      runningField.setAccessible(true);
      entryField.setAccessible(true);
      outputTableField.setAccessible(true);
      replaySourceField.setAccessible(true);
      previousRunning = runningField.getBoolean(null);
      previousEntry = (LogTable) entryField.get(null);
      previousOutput = (LogTable) outputTableField.get(null);
      previousReplaySource = replaySourceField.get(null);
      runningField.setBoolean(null, true);
      entryField.set(null, entry);
      replaySourceField.set(
          null, replay ? (org.littletonrobotics.junction.LogReplaySource) table -> true : null);
      outputTableField.set(null, entry.getSubtable(replay ? "ReplayOutputs" : "RealOutputs"));
    }

    String outputString(String key) {
      return entry.get("RealOutputs/" + key, "");
    }

    Pose3d[] outputPoses(String key) {
      return entry.get("RealOutputs/" + key, new Pose3d[0]);
    }

    @Override
    public void close() throws ReflectiveOperationException {
      replaySourceField.set(null, previousReplaySource);
      outputTableField.set(null, previousOutput);
      entryField.set(null, previousEntry);
      runningField.setBoolean(null, previousRunning);
    }
  }
}
