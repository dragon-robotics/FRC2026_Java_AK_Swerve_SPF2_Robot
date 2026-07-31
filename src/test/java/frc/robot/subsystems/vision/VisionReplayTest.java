package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.lang.reflect.Field;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogReplaySource;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

class VisionReplayTest {
  @Test
  void replayRestoresGeneratedInputsFromFixedConfiguredKeysWithoutPhotonObjects()
      throws ReflectiveOperationException {
    VisionIO.NoOp[] streams = {
      new VisionIO.NoOp("front"),
      new VisionIO.NoOp("right"),
      new VisionIO.NoOp("rear"),
      new VisionIO.NoOp("left")
    };
    VisionTest.RecordingDrive drive = new VisionTest.RecordingDrive();

    try (ReplayLoggerHarness logger = new ReplayLoggerHarness()) {
      for (int index = 0; index < streams.length; index++) {
        VisionIOInputsAutoLogged payload = new VisionIOInputsAutoLogged();
        payload.cameraName = "payload-name-" + index;
        payload.connected = true;
        payload.setPoseObservations(
            new PoseObservation[] {VisionTest.observation(70.0 + index * 0.005, 1.0, 1.0)});
        payload.setTagIds(new int[] {1});
        payload.toLog(logger.entry.getSubtable("Vision/" + streams[index].getCameraName()));

        VisionIOInputsAutoLogged decoy = new VisionIOInputsAutoLogged();
        decoy.cameraName = "decoy";
        decoy.connected = false;
        decoy.setPoseObservations(new PoseObservation[0]);
        decoy.toLog(logger.entry.getSubtable("Vision/payload-name-" + index));
      }

      Vision vision =
          new Vision(
              drive.bindings(),
              VisionRuntimeConfig.fromSystemProperties(),
              () -> false,
              () -> 70.02,
              () -> {},
              true,
              streams);
      vision.periodic();

      Field ioField = Vision.class.getDeclaredField("io");
      ioField.setAccessible(true);
      VisionIO[] retained = (VisionIO[]) ioField.get(vision);
      Field inputsField = Vision.class.getDeclaredField("inputs");
      inputsField.setAccessible(true);
      VisionIOInputsAutoLogged[] restored = (VisionIOInputsAutoLogged[]) inputsField.get(vision);
      assertAll(
          () -> assertEquals(1, drive.measurements.size()),
          () ->
              assertEquals(
                  new Pose2d(1.0, 1.0, Pose2d.kZero.getRotation()),
                  drive.measurements.get(0).pose()),
          () -> assertEquals("front", logger.output("Vision/Consensus/SelectedCamera")),
          () -> assertEquals(4, retained.length),
          () -> assertSame(streams[0], retained[0]),
          () -> assertSame(streams[1], retained[1]),
          () -> assertSame(streams[2], retained[2]),
          () -> assertSame(streams[3], retained[3]),
          () -> assertEquals("payload-name-0", restored[0].cameraName),
          () -> assertEquals("payload-name-1", restored[1].cameraName),
          () -> assertEquals("payload-name-2", restored[2].cameraName),
          () -> assertEquals("payload-name-3", restored[3].cameraName),
          () -> assertTrue(List.of(retained).stream().allMatch(VisionIO.NoOp.class::isInstance)),
          () ->
              assertTrue(
                  List.of(retained).stream()
                      .map(Object::getClass)
                      .map(Class::getName)
                      .noneMatch(
                          name ->
                              name.contains("PhotonCamera")
                                  || name.contains("PhotonCameraSim")
                                  || name.contains("VisionSystemSim"))));
    }
  }

  private static final class ReplayLoggerHarness implements AutoCloseable {
    private final Field running = Logger.class.getDeclaredField("running");
    private final Field entryField = Logger.class.getDeclaredField("entry");
    private final Field outputTable = Logger.class.getDeclaredField("outputTable");
    private final Field replaySource = Logger.class.getDeclaredField("replaySource");
    private final boolean previousRunning;
    private final LogTable previousEntry;
    private final LogTable previousOutput;
    private final LogReplaySource previousReplay;
    final LogTable entry = new LogTable(0);

    ReplayLoggerHarness() throws ReflectiveOperationException {
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
      replaySource.set(null, (LogReplaySource) table -> true);
      outputTable.set(null, entry.getSubtable("ReplayOutputs"));
    }

    String output(String key) {
      return entry.get("ReplayOutputs/" + key, "");
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
