package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import java.util.List;

/** Shared camera configuration and threshold values for the vision subsystem. */
public final class VisionConstants {
  public record CameraConfig(String name, Transform3d robotToCamera, double stdDevFactor) {}

  public static final List<CameraConfig> CAMERAS =
      List.of(
          new CameraConfig(
              "AprilTagPoseEstCameraF", transform(-11.152, -7.579, 20.930, 0.0, -15.0, 0.0), 1.0),
          new CameraConfig(
              "AprilTagPoseEstCameraR", transform(-8.387, -13.355, 15.931, 0.0, -12.0, -90.0), 1.0),
          new CameraConfig(
              "AprilTagPoseEstCameraB", transform(-9.164, 12.500, 20.839, 0.0, -15.0, 180.0), 1.0),
          new CameraConfig(
              "AprilTagPoseEstCameraL", transform(-8.387, 13.355, 15.931, 0.0, -12.0, 90.0), 1.0));

  public static final double MAX_AMBIGUITY = 0.2;
  public static final double MAX_Z_ERROR_METERS = 0.5;
  public static final double MAX_TARGET_PREFILTER_DISTANCE_METERS = 8.0;
  public static final double MAX_CONFIDENCE_DISTANCE_METERS = 5.5;
  public static final double MAX_POSE_INNOVATION_METERS = 2.5;
  public static final double CONSENSUS_RADIUS_METERS = 0.45;
  public static final double MAX_FUTURE_TIMESTAMP_SECONDS = 0.020;
  public static final double MAX_OBSERVATION_AGE_SECONDS = 0.500;
  public static final double MAX_CONSENSUS_TIMESTAMP_SPAN_SECONDS = 0.040;
  public static final double SNAPSHOT_MAX_AGE_SECONDS = 0.5;
  public static final double DISABLED_AUTO_RESEED_MIN_INTERVAL_SECONDS = 0.5;
  public static final double DISABLED_AUTO_RESEED_DELTA_METERS = 0.25;
  public static final int DISABLED_AUTO_RESEED_MIN_TAG_COUNT = 2;
  public static final int MULTITAG_INIT_STABLE_POSES_REQUIRED = 5;
  public static final double MULTITAG_INIT_MAX_TRANSLATION_DELTA_METERS = 0.20;
  public static final double MULTITAG_INIT_MAX_HEADING_DELTA_DEGREES = 10.0;
  public static final double LINEAR_STD_DEV_BASELINE = 0.02;
  public static final double HEADING_STD_DEV = 1e9;
  public static final double AIMING_STD_DEV_FACTOR = 0.6;
  public static final double VULNERABLE_GEOMETRY_STD_DEV_FACTOR = 5.0;
  public static final double CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC = 0.5;
  public static final double TRIG_MAX_ANGULAR_RATE_RAD_PER_SEC = 1.0;
  public static final double HYBRID_TRANSLATION_SPEED_THRESHOLD_MPS = 0.5;
  public static final double CONSTRAINED_HEADING_SCALE_FACTOR = 0.2;
  public static final double COPLANAR_ANGLE_THRESHOLD_DEGREES = 15.0;
  public static final int SIM_CAMERA_WIDTH_PIXELS = 800;
  public static final int SIM_CAMERA_HEIGHT_PIXELS = 600;
  public static final double SIM_CAMERA_DIAGONAL_FOV_DEGREES = 72.0;
  public static final double SIM_CAMERA_CALIBRATION_ERROR_MEAN = 0.38;
  public static final double SIM_CAMERA_CALIBRATION_ERROR_STD_DEV = 0.1;
  public static final double SIM_CAMERA_FPS = 60.0;
  public static final double SIM_CAMERA_AVERAGE_LATENCY_MS = 10.0;
  public static final double SIM_CAMERA_LATENCY_STD_DEV_MS = 5.0;
  public static final StartupStrategyOrder DEFAULT_STARTUP_STRATEGY_ORDER =
      StartupStrategyOrder.CONSTRAINED_SECOND;

  private VisionConstants() {}

  private static Transform3d transform(
      double xInches,
      double yInches,
      double zInches,
      double rollDegrees,
      double pitchDegrees,
      double yawDegrees) {
    return new Transform3d(
        new Translation3d(
            Units.inchesToMeters(xInches),
            Units.inchesToMeters(yInches),
            Units.inchesToMeters(zInches)),
        new Rotation3d(
            Units.degreesToRadians(rollDegrees),
            Units.degreesToRadians(pitchDegrees),
            Units.degreesToRadians(yawDegrees)));
  }
}
