package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

class VisionIOTest {
  @Test
  void poseObservationCopiesTagIdsInBothDirections() {
    int[] sourceTagIds = {3, 7};
    VisionIO.PoseObservation observation = observation(sourceTagIds);

    sourceTagIds[0] = 99;
    int[] accessorTagIds = observation.tagIds();
    accessorTagIds[1] = 99;

    assertArrayEquals(new int[] {3, 7}, observation.tagIds());
  }

  @Test
  void changingAnAcceptedInputArrayDoesNotMutateItsObservation() {
    int[] acceptedTagIds = {2, 5};
    VisionIO.PoseObservation observation = observation(acceptedTagIds);

    acceptedTagIds[0] = 42;

    assertArrayEquals(new int[] {2, 5}, observation.tagIds());
  }

  @Test
  void noOpOverwritesEveryStaleCameraField() {
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    inputs.cameraName = "stale";
    inputs.connected = true;
    inputs.latestTargetObservation =
        new VisionIO.TargetObservation(Rotation2d.fromDegrees(8.0), Rotation2d.fromDegrees(-3.0));
    inputs.setPoseObservations(new VisionIO.PoseObservation[] {observation(new int[] {1})});

    new VisionIO.NoOp("camera").updateInputs(inputs);

    assertAll(
        () -> assertEquals("camera", inputs.cameraName),
        () -> assertFalse(inputs.connected),
        () -> assertEquals(Rotation2d.kZero, inputs.latestTargetObservation.tx()),
        () -> assertEquals(Rotation2d.kZero, inputs.latestTargetObservation.ty()),
        () -> assertEquals(0, inputs.getPoseObservations().length),
        () -> assertEquals(0, inputs.poseObservationTagIds.length));
  }

  @Test
  void autoLoggedInputsRoundTripPoseObservationArraysThroughLogTable() {
    VisionIO.PoseObservation observation = observation(new int[] {3, 7});
    VisionIOInputsAutoLogged source = new VisionIOInputsAutoLogged();
    source.cameraName = "AprilTagPoseEstCameraF";
    source.setPoseObservations(new VisionIO.PoseObservation[] {observation});
    LogTable table = new LogTable(0);
    source.toLog(table);

    VisionIOInputsAutoLogged replayed = new VisionIOInputsAutoLogged();
    replayed.fromLog(table);

    VisionIO.PoseObservation replayedObservation = replayed.getPoseObservations()[0];
    assertAll(
        () -> assertEquals(observation.timestampSeconds(), replayedObservation.timestampSeconds()),
        () -> assertEquals(observation.pose(), replayedObservation.pose()),
        () -> assertEquals(observation.ambiguity(), replayedObservation.ambiguity()),
        () -> assertEquals(observation.tagCount(), replayedObservation.tagCount()),
        () ->
            assertEquals(
                observation.confidenceDistanceMeters(),
                replayedObservation.confidenceDistanceMeters()),
        () -> assertEquals(observation.type(), replayedObservation.type()),
        () -> assertEquals(observation.strategy(), replayedObservation.strategy()),
        () -> assertArrayEquals(new int[] {3, 7}, replayedObservation.tagIds()));
  }

  @Test
  void missingOrMismatchedTagIdSidecarRowsBecomeEmptyWithoutChangingOtherObservations() {
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    VisionIO.PoseObservation first = observation(new int[] {1});
    VisionIO.PoseObservation second = observation(new int[] {2, 3});
    inputs.setPoseObservations(new VisionIO.PoseObservation[] {first, second});
    inputs.poseObservationTagIds = new int[][] {{9}};

    VisionIO.PoseObservation[] reconstructed = inputs.getPoseObservations();

    assertAll(
        () -> assertEquals(first.timestampSeconds(), reconstructed[0].timestampSeconds()),
        () -> assertArrayEquals(new int[] {9}, reconstructed[0].tagIds()),
        () -> assertEquals(second.pose(), reconstructed[1].pose()),
        () -> assertEquals(second.strategy(), reconstructed[1].strategy()),
        () -> assertArrayEquals(new int[0], reconstructed[1].tagIds()));
  }

  private static VisionIO.PoseObservation observation(int[] tagIds) {
    return new VisionIO.PoseObservation(
        4.25,
        new Pose3d(1.0, 2.0, 3.0, new Rotation3d(0.1, 0.2, 0.3)),
        0.05,
        2,
        1.5,
        VisionIO.PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
        VisionIO.PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
        tagIds);
  }
}
