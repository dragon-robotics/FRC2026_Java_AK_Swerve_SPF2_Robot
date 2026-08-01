package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class VisionConstructionPlanTest {
  private static final List<CameraConfig> EXPECTED_CAMERAS =
      List.of(
          camera("AprilTagPoseEstCameraF", -11.152, -7.579, 20.930, 0.0, -15.0, 0.0),
          camera("AprilTagPoseEstCameraR", -8.387, -13.355, 15.931, 0.0, -12.0, -90.0),
          camera("AprilTagPoseEstCameraB", -9.164, 12.500, 20.839, 0.0, -15.0, 180.0),
          camera("AprilTagPoseEstCameraL", -8.387, 13.355, 15.931, 0.0, -12.0, 90.0));

  @Test
  void mapsEveryRuntimeModeWithoutRealizingAnyIo() {
    assertAll(
        () ->
            assertEquals(
                VisionConstructionPlan.IoKind.REAL_PHOTON,
                VisionConstructionPlan.forMode(Mode.REAL).ioKind()),
        () ->
            assertEquals(
                VisionConstructionPlan.IoKind.SIM_PHOTON,
                VisionConstructionPlan.forMode(Mode.SIM).ioKind()),
        () ->
            assertEquals(
                VisionConstructionPlan.IoKind.REPLAY_NOOP,
                VisionConstructionPlan.forMode(Mode.REPLAY).ioKind()));
  }

  @Test
  void everyModeUsesTheExactOrderedFourCameraGeometry() {
    for (Mode mode : Mode.values()) {
      assertEquals(EXPECTED_CAMERAS, VisionConstructionPlan.forMode(mode).cameras());
    }
  }

  @Test
  void defensivelyCopiesAndExposesAnImmutableCameraList() {
    var mutableCameras = new ArrayList<>(EXPECTED_CAMERAS);
    var plan =
        new VisionConstructionPlan(VisionConstructionPlan.IoKind.REPLAY_NOOP, mutableCameras);

    mutableCameras.clear();

    assertEquals(EXPECTED_CAMERAS, plan.cameras());
    assertThrows(UnsupportedOperationException.class, () -> plan.cameras().clear());
  }

  private static CameraConfig camera(
      String name,
      double xInches,
      double yInches,
      double zInches,
      double rollDegrees,
      double pitchDegrees,
      double yawDegrees) {
    return new CameraConfig(
        name,
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(xInches),
                Units.inchesToMeters(yInches),
                Units.inchesToMeters(zInches)),
            new Rotation3d(
                Units.degreesToRadians(rollDegrees),
                Units.degreesToRadians(pitchDegrees),
                Units.degreesToRadians(yawDegrees))),
        1.0);
  }
}
