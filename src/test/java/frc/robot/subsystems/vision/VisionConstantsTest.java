package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.junit.jupiter.api.Test;

class VisionConstantsTest {
  @Test
  void cameraConfigsPreserveTheLockedCadOrderTransformsAndFactors() {
    assertEquals(
        List.of(
            "AprilTagPoseEstCameraF",
            "AprilTagPoseEstCameraR",
            "AprilTagPoseEstCameraB",
            "AprilTagPoseEstCameraL"),
        VisionConstants.CAMERAS.stream().map(VisionConstants.CameraConfig::name).toList());

    assertTransform(
        VisionConstants.CAMERAS.get(0).robotToCamera(),
        -0.2832608,
        -0.1925066,
        0.5316220,
        0.0,
        -15.0,
        0.0);
    assertTransform(
        VisionConstants.CAMERAS.get(1).robotToCamera(),
        -0.2130298,
        -0.3392170,
        0.4046474,
        0.0,
        -12.0,
        -90.0);
    assertTransform(
        VisionConstants.CAMERAS.get(2).robotToCamera(),
        -0.2327656,
        0.3175000,
        0.5293106,
        0.0,
        -15.0,
        180.0);
    assertTransform(
        VisionConstants.CAMERAS.get(3).robotToCamera(),
        -0.2130298,
        0.3392170,
        0.4046474,
        0.0,
        -12.0,
        90.0);
    assertEquals(
        List.of(1.0, 1.0, 1.0, 1.0),
        VisionConstants.CAMERAS.stream().map(VisionConstants.CameraConfig::stdDevFactor).toList());
  }

  @Test
  void tuningConstantsPreserveTheApprovedVisionContract() {
    assertAll(
        () -> assertEquals(0.2, VisionConstants.MAX_AMBIGUITY, 1e-12),
        () -> assertEquals(0.5, VisionConstants.MAX_Z_ERROR_METERS, 1e-12),
        () -> assertEquals(8.0, VisionConstants.MAX_TARGET_PREFILTER_DISTANCE_METERS, 1e-12),
        () -> assertEquals(5.5, VisionConstants.MAX_CONFIDENCE_DISTANCE_METERS, 1e-12),
        () -> assertEquals(2.5, VisionConstants.MAX_POSE_INNOVATION_METERS, 1e-12),
        () -> assertEquals(0.45, VisionConstants.CONSENSUS_RADIUS_METERS, 1e-12),
        () -> assertEquals(0.020, VisionConstants.MAX_FUTURE_TIMESTAMP_SECONDS, 1e-12),
        () -> assertEquals(0.500, VisionConstants.MAX_OBSERVATION_AGE_SECONDS, 1e-12),
        () -> assertEquals(0.040, VisionConstants.MAX_CONSENSUS_TIMESTAMP_SPAN_SECONDS, 1e-12),
        () -> assertEquals(0.5, VisionConstants.SNAPSHOT_MAX_AGE_SECONDS, 1e-12),
        () -> assertEquals(0.5, VisionConstants.DISABLED_AUTO_RESEED_MIN_INTERVAL_SECONDS, 1e-12),
        () -> assertEquals(0.25, VisionConstants.DISABLED_AUTO_RESEED_DELTA_METERS, 1e-12),
        () -> assertEquals(2, VisionConstants.DISABLED_AUTO_RESEED_MIN_TAG_COUNT),
        () -> assertEquals(5, VisionConstants.MULTITAG_INIT_STABLE_POSES_REQUIRED),
        () -> assertEquals(0.20, VisionConstants.MULTITAG_INIT_MAX_TRANSLATION_DELTA_METERS, 1e-12),
        () -> assertEquals(10.0, VisionConstants.MULTITAG_INIT_MAX_HEADING_DELTA_DEGREES, 1e-12),
        () -> assertEquals(0.02, VisionConstants.LINEAR_STD_DEV_BASELINE, 1e-12),
        () -> assertEquals(1e9, VisionConstants.HEADING_STD_DEV, 1.0),
        () -> assertEquals(0.6, VisionConstants.AIMING_STD_DEV_FACTOR, 1e-12),
        () -> assertEquals(5.0, VisionConstants.VULNERABLE_GEOMETRY_STD_DEV_FACTOR, 1e-12),
        () -> assertEquals(0.5, VisionConstants.CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC, 1e-12),
        () -> assertEquals(1.0, VisionConstants.TRIG_MAX_ANGULAR_RATE_RAD_PER_SEC, 1e-12),
        () -> assertEquals(0.5, VisionConstants.HYBRID_TRANSLATION_SPEED_THRESHOLD_MPS, 1e-12),
        () -> assertEquals(0.2, VisionConstants.CONSTRAINED_HEADING_SCALE_FACTOR, 1e-12),
        () -> assertEquals(15.0, VisionConstants.COPLANAR_ANGLE_THRESHOLD_DEGREES, 1e-12),
        () -> assertEquals(800, VisionConstants.SIM_CAMERA_WIDTH_PIXELS),
        () -> assertEquals(600, VisionConstants.SIM_CAMERA_HEIGHT_PIXELS),
        () -> assertEquals(72.0, VisionConstants.SIM_CAMERA_DIAGONAL_FOV_DEGREES, 1e-12),
        () -> assertEquals(0.38, VisionConstants.SIM_CAMERA_CALIBRATION_ERROR_MEAN, 1e-12),
        () -> assertEquals(0.1, VisionConstants.SIM_CAMERA_CALIBRATION_ERROR_STD_DEV, 1e-12),
        () -> assertEquals(60.0, VisionConstants.SIM_CAMERA_FPS, 1e-12),
        () -> assertEquals(10.0, VisionConstants.SIM_CAMERA_AVERAGE_LATENCY_MS, 1e-12),
        () -> assertEquals(5.0, VisionConstants.SIM_CAMERA_LATENCY_STD_DEV_MS, 1e-12),
        () ->
            assertEquals(
                VisionRuntimeConfig.StartupStrategyOrder.CONSTRAINED_SECOND,
                VisionConstants.DEFAULT_STARTUP_STRATEGY_ORDER));
  }

  private static void assertTransform(
      Transform3d transform,
      double xMeters,
      double yMeters,
      double zMeters,
      double rollDegrees,
      double pitchDegrees,
      double yawDegrees) {
    assertAll(
        () -> assertEquals(xMeters, transform.getX(), 1e-9),
        () -> assertEquals(yMeters, transform.getY(), 1e-9),
        () -> assertEquals(zMeters, transform.getZ(), 1e-9),
        () -> assertEquals(Math.toRadians(rollDegrees), transform.getRotation().getX(), 1e-9),
        () -> assertEquals(Math.toRadians(pitchDegrees), transform.getRotation().getY(), 1e-9),
        () -> assertEquals(Math.toRadians(yawDegrees), transform.getRotation().getZ(), 1e-9));
  }
}
