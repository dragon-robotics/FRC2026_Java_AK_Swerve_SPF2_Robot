package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.PoseSolveStrategy;
import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import frc.robot.subsystems.vision.VisionRuntimeConfig.TagDistanceConfidenceMode;
import frc.robot.util.constants.FieldConstants;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogReplaySource;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

class VisionLoggingTest {
  @Test
  void populatedThenEmptyLoopWritesAndClearsTheCompleteTaskSixKeyContract()
      throws ReflectiveOperationException {
    VisionRuntimeConfig config =
        new VisionRuntimeConfig(
            8.0,
            false,
            "HYBRID",
            List.of(PoseSolveStrategy.CONSTRAINED_SOLVEPNP),
            true,
            TagDistanceConfidenceMode.MAX_TAG_DISTANCE,
            StartupStrategyOrder.REFERENCE);
    VisionTest.RecordingDrive drive = new VisionTest.RecordingDrive();
    drive.currentPose = new Pose2d(0.0, 0.0, Pose2d.kZero.getRotation());
    drive.timestampedPose = Optional.of(drive.currentPose);
    drive.speeds = new ChassisSpeeds(3.0, 4.0, 1.25);
    drive.pitch = 2.5;
    drive.roll = -3.5;
    PoseObservation accepted = VisionTest.observation(99.98, 4.0, 3.0);
    PoseObservation superseded = VisionTest.observation(99.96, 4.1, 3.0);
    PoseObservation skewed = VisionTest.observation(99.49, 4.2, 3.0);
    PoseObservation rejected =
        new PoseObservation(
            99.97,
            new Pose3d(4.3, 3.0, 0.0, Pose3d.kZero.getRotation()),
            0.4,
            1,
            2.0,
            PoseObservationType.PHOTONVISION,
            PoseSolveStrategy.LOWEST_AMBIGUITY,
            new int[] {2});
    VisionTest.ScriptedIO camera =
        new VisionTest.ScriptedIO(
                "front",
                true,
                new PoseObservation[] {superseded, accepted, skewed, rejected},
                new PoseObservation[0])
            .withAggregateTagIds(26, 999999);

    try (LoggerHarness logger = new LoggerHarness()) {
      Vision vision =
          new Vision(drive.bindings(), config, () -> true, () -> 100.0, () -> {}, true, camera);
      vision.periodic();

      String cameraKey = "Vision/front/";
      assertAll(
          () -> assertEquals(4, logger.poses(cameraKey + "RawRobotPoses").length),
          () -> assertEquals(2, logger.poses(cameraKey + "AcceptedRobotPoses").length),
          () -> assertEquals(1, logger.poses(cameraKey + "RejectedRobotPoses").length),
          () ->
              assertArrayEquals(
                  new Pose3d[] {superseded.pose()},
                  logger.poses(cameraKey + "SupersededRobotPoses")),
          () ->
              assertArrayEquals(
                  new Pose3d[] {skewed.pose()}, logger.poses(cameraKey + "SkewRejectedRobotPoses")),
          () ->
              assertArrayEquals(
                  new Pose3d[] {accepted.pose()}, logger.poses(cameraKey + "SelectedRobotPoses")),
          () -> assertEquals(1, logger.poses(cameraKey + "TagPoses").length),
          () -> assertArrayEquals(new int[] {26, 999999}, logger.ints(cameraKey + "TagIDs")),
          () ->
              assertEquals(
                  PoseSolveStrategy.LOWEST_AMBIGUITY.name(),
                  logger.string(cameraKey + "ActiveStrategy")),
          () ->
              assertEquals(
                  PoseObservationType.PHOTONVISION.name(),
                  logger.string(cameraKey + "ObservationType")),
          () ->
              assertEquals(
                  VisionFilter.HIGH_AMBIGUITY, logger.string(cameraKey + "RejectionReason")),
          () -> assertEquals(2.0, logger.number(cameraKey + "ConfidenceDistanceMeters"), 0.0),
          () -> assertEquals(0.4, logger.number(cameraKey + "Ambiguity"), 0.0),
          () -> assertEquals(99.97, logger.number(cameraKey + "CaptureTimestampSeconds"), 0.0),
          () -> assertEquals(0.03, logger.number(cameraKey + "TimestampSkewSeconds"), 1e-12),
          () -> assertEquals(2.5, logger.number(cameraKey + "PitchDegrees"), 0.0),
          () -> assertEquals(-3.5, logger.number(cameraKey + "RollDegrees"), 0.0),
          () -> assertTrue(logger.number(cameraKey + "InnovationMeters") > 0.0),
          () -> assertTrue(logger.number(cameraKey + "InnovationBypassedInDisabledMeters") > 2.5),
          () -> assertEquals(4, logger.poses("Vision/RawRobotPoses").length),
          () -> assertEquals(2, logger.poses("Vision/AcceptedRobotPoses").length),
          () -> assertEquals(3, logger.poses("Vision/RejectedRobotPoses").length),
          () ->
              assertArrayEquals(
                  new Pose3d[] {accepted.pose()}, logger.poses("Vision/SelectedRobotPoses")),
          () -> assertEquals("front", logger.string("Vision/Consensus/SelectedCamera")),
          () -> assertEquals(2, logger.integer("Vision/Consensus/CandidateCount")),
          () -> assertEquals(1, logger.integer("Vision/Consensus/SelectedClusterSize")),
          () -> assertEquals(0.04, logger.number("Vision/Consensus/SelectedStdDevMeters"), 1e-12),
          () -> assertTrue(logger.number("Vision/Consensus/SelectedInnovationMeters") > 0.0),
          () ->
              assertEquals(99.98, logger.number("Vision/Consensus/SelectedTimestampSeconds"), 0.0),
          () -> assertFalse(logger.bool("Vision/Aiming")),
          () -> assertEquals(5.0, logger.number("Vision/LinearSpeedMetersPerSecond"), 0.0),
          () -> assertEquals(1.25, logger.number("Vision/AngularSpeedRadiansPerSecond"), 0.0),
          () ->
              assertArrayEquals(
                  new Pose3d[] {accepted.pose()}, logger.poses("Vision/AcceptedFieldPoses")),
          () -> assertEquals(2, logger.poses("Vision/AcceptedTagToPoseLines").length),
          () -> assertTrue(logger.number("Vision/LoopExecutionMilliseconds") >= 0.0),
          () -> assertEquals("HYBRID", logger.string("Vision/Config/StrategyMode")),
          () -> assertEquals("REFERENCE", logger.string("Vision/Config/StartupStrategyOrder")),
          () ->
              assertEquals(
                  "CONSTRAINED_SOLVEPNP", logger.string("Vision/Config/ExplicitStrategyOrder")),
          () ->
              assertEquals(
                  "MAX_TAG_DISTANCE", logger.string("Vision/Config/TagDistanceConfidenceMode")));

      drive.speeds = new ChassisSpeeds();
      drive.pitch = 0.0;
      drive.roll = 0.0;
      vision.periodic();

      assertAll(
          () -> assertEquals(0, logger.poses(cameraKey + "RawRobotPoses").length),
          () -> assertEquals(0, logger.poses(cameraKey + "AcceptedRobotPoses").length),
          () -> assertEquals(0, logger.poses(cameraKey + "RejectedRobotPoses").length),
          () -> assertEquals(0, logger.poses(cameraKey + "SupersededRobotPoses").length),
          () -> assertEquals(0, logger.poses(cameraKey + "SkewRejectedRobotPoses").length),
          () -> assertEquals(0, logger.poses(cameraKey + "SelectedRobotPoses").length),
          () -> assertEquals(0, logger.poses(cameraKey + "TagPoses").length),
          () -> assertArrayEquals(new int[0], logger.ints(cameraKey + "TagIDs")),
          () -> assertEquals("", logger.string(cameraKey + "ActiveStrategy")),
          () -> assertEquals("", logger.string(cameraKey + "ObservationType")),
          () -> assertEquals("", logger.string(cameraKey + "RejectionReason")),
          () -> assertEquals(0.0, logger.number(cameraKey + "ConfidenceDistanceMeters"), 0.0),
          () -> assertEquals(0.0, logger.number(cameraKey + "Ambiguity"), 0.0),
          () -> assertEquals(0.0, logger.number(cameraKey + "CaptureTimestampSeconds"), 0.0),
          () -> assertEquals(0.0, logger.number(cameraKey + "TimestampSkewSeconds"), 0.0),
          () -> assertEquals(0.0, logger.number(cameraKey + "PitchDegrees"), 0.0),
          () -> assertEquals(0.0, logger.number(cameraKey + "RollDegrees"), 0.0),
          () -> assertEquals(0.0, logger.number(cameraKey + "InnovationMeters"), 0.0),
          () ->
              assertEquals(
                  0.0, logger.number(cameraKey + "InnovationBypassedInDisabledMeters"), 0.0),
          () -> assertEquals(0, logger.poses("Vision/RawRobotPoses").length),
          () -> assertEquals(0, logger.poses("Vision/AcceptedRobotPoses").length),
          () -> assertEquals(0, logger.poses("Vision/RejectedRobotPoses").length),
          () -> assertEquals(0, logger.poses("Vision/SelectedRobotPoses").length),
          () -> assertEquals("", logger.string("Vision/Consensus/SelectedCamera")),
          () -> assertEquals(0, logger.integer("Vision/Consensus/CandidateCount")),
          () -> assertEquals(0, logger.integer("Vision/Consensus/SelectedClusterSize")),
          () -> assertEquals(0.0, logger.number("Vision/Consensus/SelectedStdDevMeters"), 0.0),
          () -> assertEquals(0.0, logger.number("Vision/Consensus/SelectedInnovationMeters"), 0.0),
          () -> assertEquals(0.0, logger.number("Vision/Consensus/SelectedTimestampSeconds"), 0.0),
          () -> assertFalse(logger.bool("Vision/Aiming")),
          () -> assertEquals(0.0, logger.number("Vision/LinearSpeedMetersPerSecond"), 0.0),
          () -> assertEquals(0.0, logger.number("Vision/AngularSpeedRadiansPerSecond"), 0.0),
          () -> assertEquals(0, logger.poses("Vision/AcceptedFieldPoses").length),
          () -> assertEquals(0, logger.poses("Vision/AcceptedTagToPoseLines").length),
          () -> assertTrue(logger.number("Vision/LoopExecutionMilliseconds") >= 0.0),
          () -> assertEquals("HYBRID", logger.string("Vision/Config/StrategyMode")),
          () -> assertEquals("REFERENCE", logger.string("Vision/Config/StartupStrategyOrder")),
          () ->
              assertEquals(
                  "CONSTRAINED_SOLVEPNP", logger.string("Vision/Config/ExplicitStrategyOrder")),
          () ->
              assertEquals(
                  "MAX_TAG_DISTANCE", logger.string("Vision/Config/TagDistanceConfidenceMode")));
    }
  }

  @Test
  void selectedObservationIdsDriveAcceptedLinesWhileAggregateIdsDriveCameraOverlay()
      throws ReflectiveOperationException {
    VisionTest.RecordingDrive drive = new VisionTest.RecordingDrive();
    PoseObservation superseded =
        new PoseObservation(
            5.0,
            new Pose3d(1.9, 2.0, 0.0, Pose3d.kZero.getRotation()),
            0.05,
            1,
            2.0,
            PoseObservationType.PHOTONVISION,
            PoseSolveStrategy.LOWEST_AMBIGUITY,
            new int[] {26});
    PoseObservation selected =
        new PoseObservation(
            5.01,
            new Pose3d(2.0, 2.0, 0.0, Pose3d.kZero.getRotation()),
            0.05,
            2,
            2.0,
            PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new int[] {20, 888888});
    VisionTest.ScriptedIO camera =
        new VisionTest.ScriptedIO("front", true, superseded, selected)
            .withAggregateTagIds(26, 20, 999999);

    try (LoggerHarness logger = new LoggerHarness()) {
      new Vision(
              drive.bindings(),
              VisionRuntimeConfig.fromSystemProperties(),
              () -> true,
              () -> 5.1,
              () -> {},
              true,
              camera)
          .periodic();

      assertAll(
          () -> assertArrayEquals(new int[] {26, 20, 999999}, logger.ints("Vision/front/TagIDs")),
          () ->
              assertArrayEquals(
                  new Pose3d[] {
                    FieldConstants.APTAG_FIELD_LAYOUT.getTagPose(26).orElseThrow(),
                    FieldConstants.APTAG_FIELD_LAYOUT.getTagPose(20).orElseThrow()
                  },
                  logger.poses("Vision/front/TagPoses")),
          () ->
              assertArrayEquals(
                  new Pose3d[] {
                    FieldConstants.APTAG_FIELD_LAYOUT.getTagPose(20).orElseThrow(), selected.pose()
                  },
                  logger.poses("Vision/AcceptedTagToPoseLines")));
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

    LoggerHarness() throws ReflectiveOperationException {
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

    Pose3d[] poses(String key) {
      return entry.get("RealOutputs/" + key, new Pose3d[0]);
    }

    int[] ints(String key) {
      return entry.get("RealOutputs/" + key, new int[0]);
    }

    String string(String key) {
      return entry.get("RealOutputs/" + key, "__missing__");
    }

    double number(String key) {
      return entry.get("RealOutputs/" + key, Double.NaN);
    }

    long integer(String key) {
      return entry.get("RealOutputs/" + key, Long.MIN_VALUE);
    }

    boolean bool(String key) {
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
