package frc.robot.subsystems.vision.sim;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionRuntimeConfig;
import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import frc.robot.subsystems.vision.VisionRuntimeConfig.TagDistanceConfidenceMode;
import frc.robot.subsystems.vision.VisionSimulation;
import frc.robot.subsystems.vision.VisionSimulation.CameraDiagnostics;
import frc.robot.subsystems.vision.VisionSimulationHarness;
import frc.robot.subsystems.vision.VisionSimulationHarness.HookOrderMetrics;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.UUID;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonCamera;

@Tag("vision-sim")
class VisionSimulationLifecycleTest {
  private static final VisionRuntimeConfig CONFIG =
      new VisionRuntimeConfig(
          8.0,
          true,
          "HYBRID",
          List.of(),
          false,
          TagDistanceConfidenceMode.ALL_TAG_AVERAGE,
          StartupStrategyOrder.CONSTRAINED_SECOND);

  @BeforeAll
  static void initializeNativeSimulation() {
    HAL.initialize(500, 0);
    PhotonCamera.setVersionCheckEnabled(false);
    SimHooks.pauseTiming();
  }

  @AfterAll
  static void restoreTiming() {
    NetworkTableInstance.getDefault().flush();
    SimHooks.resumeTiming();
    HAL.shutdown();
  }

  @Test
  void oneOwnerRegistersExactCamerasRejectsDuplicatesAndUpdatesOnlyFromItsHook() {
    String instanceName = "task8-lifecycle-" + UUID.randomUUID();
    AtomicInteger poseSupplierCalls = new AtomicInteger();
    Pose2d truth = new Pose2d(4.407, 7.279, Rotation2d.fromDegrees(-90.0));

    VisionSimulation owner =
        new VisionSimulation(
            instanceName,
            () -> {
              poseSupplierCalls.incrementAndGet();
              return truth;
            });
    try (owner) {
      List<VisionIOPhotonVisionSim> adapters = new ArrayList<>();
      for (CameraConfig camera : VisionConstants.CAMERAS) {
        adapters.add(
            new VisionIOPhotonVisionSim(camera, CONFIG, new FixedHeadingProvider(), owner));
      }
      for (VisionIOPhotonVisionSim adapter : adapters) {
        assertTrue(VisionSimulationHarness.usesExactOwnedCamera(owner, adapter));
      }

      List<CameraDiagnostics> diagnostics = owner.cameraDiagnostics();
      assertAll(
          () -> assertEquals(4, owner.cameraCount()),
          () -> assertEquals(4, diagnostics.size()),
          () ->
              assertThrows(
                  UnsupportedOperationException.class, () -> diagnostics.add(diagnostics.get(0))));

      for (int index = 0; index < VisionConstants.CAMERAS.size(); index++) {
        CameraConfig expected = VisionConstants.CAMERAS.get(index);
        CameraDiagnostics actual = diagnostics.get(index);
        assertAll(
            () -> assertEquals(expected.name(), actual.name()),
            () -> assertEquals(expected.robotToCamera(), actual.robotToCamera()),
            () -> assertEquals(800, actual.resolutionWidthPixels()),
            () -> assertEquals(600, actual.resolutionHeightPixels()),
            () -> assertEquals(72.0, actual.diagonalFovDegrees(), 1e-12),
            () -> assertEquals(0.38, actual.calibrationErrorMeanPixels(), 0.0),
            () -> assertEquals(0.1, actual.calibrationErrorStdDevPixels(), 0.0),
            () -> assertEquals(60.0, actual.fps(), 1e-12),
            () -> assertEquals(10.0, actual.averageLatencyMs(), 1e-12),
            () -> assertEquals(5.0, actual.latencyStdDevMs(), 1e-12),
            () -> assertEquals(100.43825454233105, actual.calibrationNoiseSampleX(), 1e-12),
            () -> assertEquals(199.81484848035254, actual.calibrationNoiseSampleY(), 1e-12));
      }

      assertThrows(
          IllegalArgumentException.class,
          () ->
              new VisionIOPhotonVisionSim(
                  VisionConstants.CAMERAS.get(0), CONFIG, new FixedHeadingProvider(), owner));

      for (VisionIOPhotonVisionSim adapter : adapters) {
        adapter.updateInputs(new VisionIOInputs());
      }
      assertAll(
          () -> assertEquals(0L, owner.updateCount()),
          () -> assertEquals(0, poseSupplierCalls.get()));

      SimHooks.stepTiming(0.020);
      owner.update();
      for (VisionIOPhotonVisionSim adapter : adapters) {
        adapter.updateInputs(new VisionIOInputs());
      }
      assertAll(
          () -> assertEquals(1L, owner.updateCount()),
          () -> assertEquals(1, poseSupplierCalls.get()));

      HookOrderMetrics hookOrder =
          VisionSimulationHarness.runHookCycle(
              owner, truth, CONFIG, adapters.toArray(VisionIO[]::new));
      assertAll(
          () -> assertEquals(2L, hookOrder.ownerUpdateCount()),
          () -> assertEquals(2L, hookOrder.firstCameraReadOwnerUpdateCount()),
          () -> assertEquals(2, poseSupplierCalls.get()));
    }
    assertAll(
        () -> assertEquals(0, owner.cameraCount()),
        () -> assertThrows(IllegalStateException.class, owner::update),
        owner::close);
    assertCameraServerResourcesGone();

    VisionSimulation replacement =
        new VisionSimulation("task8-lifecycle-replacement-" + UUID.randomUUID(), () -> truth);
    try (replacement) {
      List<VisionIOPhotonVisionSim> replacementAdapters = new ArrayList<>();
      for (CameraConfig camera : VisionConstants.CAMERAS) {
        VisionIOPhotonVisionSim adapter =
            new VisionIOPhotonVisionSim(camera, CONFIG, new FixedHeadingProvider(), replacement);
        replacementAdapters.add(adapter);
        assertTrue(VisionSimulationHarness.usesExactOwnedCamera(replacement, adapter));
      }
      SimHooks.stepTiming(0.020);
      replacement.update();
      for (VisionIOPhotonVisionSim adapter : replacementAdapters) {
        adapter.updateInputs(new VisionIOInputs());
      }
      assertEquals(1L, replacement.updateCount());
    }
    assertEquals(0, replacement.cameraCount());
    assertCameraServerResourcesGone();
  }

  private static void assertCameraServerResourcesGone() {
    for (CameraConfig camera : VisionConstants.CAMERAS) {
      assertStreamResourcesGone(camera.name() + "-raw");
      assertStreamResourcesGone(camera.name() + "-processed");
    }
  }

  private static void assertStreamResourcesGone(String streamName) {
    assertNull(CameraServer.getServer("serve_" + streamName));
    VideoException missingSource =
        assertThrows(VideoException.class, () -> CameraServer.getVideo(streamName));
    assertTrue(missingSource.getMessage().contains("could not find camera " + streamName));
  }

  private static final class FixedHeadingProvider implements VisionIOPhotonVision.HeadingProvider {
    @Override
    public Optional<Rotation2d> headingAt(double fpgaTimestampSeconds) {
      return Optional.of(Rotation2d.kZero);
    }

    @Override
    public Optional<Pose3d> seedPoseAt(double fpgaTimestampSeconds) {
      return Optional.of(Pose3d.kZero);
    }

    @Override
    public double angularRateRadPerSecond() {
      return 0.0;
    }

    @Override
    public double linearSpeedMetersPerSecond() {
      return 0.0;
    }
  }
}
