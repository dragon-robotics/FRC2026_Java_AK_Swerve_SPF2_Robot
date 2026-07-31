package frc.robot.subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.util.constants.FieldConstants;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;
import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Shared owner for every Photon camera in one simulated robot. */
public final class VisionSimulation implements AutoCloseable {
  private static final String PRODUCTION_INSTANCE_NAME = "main";
  private static final CameraSettings CAMERA_SETTINGS =
      new CameraSettings(
          VisionConstants.SIM_CAMERA_WIDTH_PIXELS,
          VisionConstants.SIM_CAMERA_HEIGHT_PIXELS,
          VisionConstants.SIM_CAMERA_DIAGONAL_FOV_DEGREES,
          VisionConstants.SIM_CAMERA_CALIBRATION_ERROR_MEAN,
          VisionConstants.SIM_CAMERA_CALIBRATION_ERROR_STD_DEV,
          VisionConstants.SIM_CAMERA_FPS,
          VisionConstants.SIM_CAMERA_AVERAGE_LATENCY_MS,
          VisionConstants.SIM_CAMERA_LATENCY_STD_DEV_MS,
          2375L);
  private static final Point CALIBRATION_NOISE_PROBE = new Point(100.0, 200.0);

  private final VisionSystemSim visionSystem;
  private final Map<String, RegisteredCamera> cameras = new LinkedHashMap<>();
  private Supplier<Pose2d> poseSupplier;
  private long updateCount;
  private boolean closed;

  /** Creates the production simulation owner using the instance name {@code main}. */
  public VisionSimulation(Supplier<Pose2d> poseSupplier) {
    this(PRODUCTION_INSTANCE_NAME, poseSupplier);
  }

  /** Creates an instance-named owner, primarily for isolated simulation processes. */
  public VisionSimulation(String instanceName, Supplier<Pose2d> poseSupplier) {
    if (instanceName == null || instanceName.isBlank()) {
      throw new IllegalArgumentException("Vision simulation instance name must not be blank");
    }
    this.poseSupplier = Objects.requireNonNull(poseSupplier);
    visionSystem = new VisionSystemSim(instanceName);
    visionSystem.addAprilTags(FieldConstants.APTAG_FIELD_LAYOUT);
  }

  void registerCamera(PhotonCamera camera, CameraConfig config) {
    requireOpen();
    Objects.requireNonNull(camera);
    Objects.requireNonNull(config);
    if (!camera.getName().equals(config.name())) {
      throw new IllegalArgumentException(
          "Photon camera name " + camera.getName() + " does not match " + config.name());
    }
    if (cameras.containsKey(config.name())) {
      throw new IllegalArgumentException("Duplicate simulated camera name: " + config.name());
    }

    SimCameraProperties properties = CAMERA_SETTINGS.createProperties();
    Point noiseSample = properties.estPixelNoise(new Point[] {CALIBRATION_NOISE_PROBE})[0];
    PhotonCameraSim cameraSim =
        new PhotonCameraSim(camera, properties, FieldConstants.APTAG_FIELD_LAYOUT);
    try {
      visionSystem.addCamera(cameraSim, config.robotToCamera());
      Transform3d registeredTransform =
          visionSystem
              .getRobotToCamera(cameraSim)
              .orElseThrow(() -> new IllegalStateException("Camera transform was not registered"));
      cameras.put(
          config.name(),
          new RegisteredCamera(
              camera,
              cameraSim,
              new CameraDiagnostics(
                  cameraSim.getCamera().getName(),
                  registeredTransform,
                  cameraSim.prop.getResWidth(),
                  cameraSim.prop.getResHeight(),
                  geometricDiagonalFovDegrees(cameraSim.prop),
                  CAMERA_SETTINGS.calibrationErrorMeanPixels(),
                  CAMERA_SETTINGS.calibrationErrorStdDevPixels(),
                  cameraSim.prop.getFPS(),
                  cameraSim.prop.getAvgLatencyMs(),
                  cameraSim.prop.getLatencyStdDevMs(),
                  noiseSample.x,
                  noiseSample.y)));
    } catch (RuntimeException | Error exception) {
      closeCameraSimulation(cameraSim);
      throw exception;
    }
  }

  /** Advances all registered cameras from one independently supplied truth pose. */
  public void update() {
    requireOpen();
    Pose2d pose = Objects.requireNonNull(poseSupplier.get(), "Vision truth pose must not be null");
    visionSystem.update(pose);
    updateCount++;
  }

  /** Replaces the independent truth source used by subsequent updates. */
  public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
    requireOpen();
    this.poseSupplier = Objects.requireNonNull(poseSupplier);
  }

  /** Returns the number of cameras currently owned by this simulation. */
  public int cameraCount() {
    return cameras.size();
  }

  /** Returns how many owner-level simulation updates have run. */
  public long updateCount() {
    return updateCount;
  }

  /**
   * Returns an immutable snapshot of registered camera transforms and actual optical properties.
   */
  public List<CameraDiagnostics> cameraDiagnostics() {
    List<CameraDiagnostics> diagnostics = new ArrayList<>(cameras.size());
    for (RegisteredCamera camera : cameras.values()) {
      diagnostics.add(camera.diagnostics());
    }
    return List.copyOf(diagnostics);
  }

  boolean hasExactCameraInstance(PhotonCamera camera) {
    RegisteredCamera registered = cameras.get(camera.getName());
    return registered != null
        && registered.camera() == camera
        && registered.cameraSim().getCamera() == camera;
  }

  @Override
  public void close() {
    if (closed) {
      return;
    }
    closed = true;
    RuntimeException failure = null;
    for (RegisteredCamera registered : cameras.values()) {
      try {
        closeCameraSimulation(registered.cameraSim());
      } catch (RuntimeException exception) {
        failure = accumulate(failure, exception);
      }
      try {
        registered.camera().close();
      } catch (RuntimeException exception) {
        failure = accumulate(failure, exception);
      }
    }
    visionSystem.clearCameras();
    cameras.clear();
    if (failure != null) {
      throw failure;
    }
  }

  private void requireOpen() {
    if (closed) {
      throw new IllegalStateException("Vision simulation is closed");
    }
  }

  private static RuntimeException accumulate(
      RuntimeException existing, RuntimeException additional) {
    if (existing == null) {
      return additional;
    }
    existing.addSuppressed(additional);
    return existing;
  }

  private static double geometricDiagonalFovDegrees(SimCameraProperties properties) {
    // PhotonLib 2026.3.4 FOV getters include width/height, one pixel beyond the last valid center.
    // Reconstruct from registered intrinsics and valid pixel-center extents; getDiagFOV() also
    // takes
    // a hypot of angles rather than the geometric diagonal.
    double horizontalTangent =
        (properties.getResWidth() - 1.0) / (2.0 * properties.getIntrinsics().get(0, 0));
    double verticalTangent =
        (properties.getResHeight() - 1.0) / (2.0 * properties.getIntrinsics().get(1, 1));
    return Math.toDegrees(2.0 * Math.atan(Math.hypot(horizontalTangent, verticalTangent)));
  }

  private static void closeCameraSimulation(PhotonCameraSim cameraSim) {
    String rawName = cameraSim.getCamera().getName() + "-raw";
    String processedName = cameraSim.getCamera().getName() + "-processed";
    RuntimeException failure = null;
    try {
      closeServer("serve_" + rawName);
    } catch (RuntimeException exception) {
      failure = accumulate(failure, exception);
    }
    try {
      closeServer("serve_" + processedName);
    } catch (RuntimeException exception) {
      failure = accumulate(failure, exception);
    }
    try {
      CameraServer.removeCamera(rawName);
    } catch (RuntimeException exception) {
      failure = accumulate(failure, exception);
    }
    try {
      CameraServer.removeCamera(processedName);
    } catch (RuntimeException exception) {
      failure = accumulate(failure, exception);
    }
    try {
      cameraSim.close();
    } catch (RuntimeException exception) {
      failure = accumulate(failure, exception);
    }
    if (failure != null) {
      throw failure;
    }
  }

  private static void closeServer(String name) {
    VideoSink server = CameraServer.getServer(name);
    if (server != null) {
      CameraServer.removeServer(name);
      server.close();
    }
  }

  /** Immutable registered-camera diagnostics without exposing mutable Photon simulation objects. */
  public record CameraDiagnostics(
      String name,
      Transform3d robotToCamera,
      int resolutionWidthPixels,
      int resolutionHeightPixels,
      double diagonalFovDegrees,
      double calibrationErrorMeanPixels,
      double calibrationErrorStdDevPixels,
      double fps,
      double averageLatencyMs,
      double latencyStdDevMs,
      double calibrationNoiseSampleX,
      double calibrationNoiseSampleY) {}

  private record RegisteredCamera(
      PhotonCamera camera, PhotonCameraSim cameraSim, CameraDiagnostics diagnostics) {}

  private record CameraSettings(
      int resolutionWidthPixels,
      int resolutionHeightPixels,
      double diagonalFovDegrees,
      double calibrationErrorMeanPixels,
      double calibrationErrorStdDevPixels,
      double fps,
      double averageLatencyMs,
      double latencyStdDevMs,
      long randomSeed) {
    SimCameraProperties createProperties() {
      return new SimCameraProperties()
          .setRandomSeed(randomSeed)
          .setCalibration(
              resolutionWidthPixels,
              resolutionHeightPixels,
              Rotation2d.fromDegrees(diagonalFovDegrees))
          .setCalibError(calibrationErrorMeanPixels, calibrationErrorStdDevPixels)
          .setFPS(fps)
          .setAvgLatencyMs(averageLatencyMs)
          .setLatencyStdDevMs(latencyStdDevMs);
    }
  }
}
