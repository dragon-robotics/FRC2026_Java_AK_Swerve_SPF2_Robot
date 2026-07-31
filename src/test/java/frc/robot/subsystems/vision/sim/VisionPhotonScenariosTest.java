package frc.robot.subsystems.vision.sim;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.vision.VisionIO.PoseSolveStrategy;
import frc.robot.subsystems.vision.VisionRuntimeConfig;
import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import frc.robot.subsystems.vision.VisionRuntimeConfig.TagDistanceConfidenceMode;
import frc.robot.subsystems.vision.VisionSimulationHarness;
import frc.robot.subsystems.vision.VisionSimulationHarness.ScenarioMetrics;
import java.util.List;
import java.util.UUID;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonCamera;

@Tag("vision-sim")
class VisionPhotonScenariosTest {
  private static final int WARMUP_CYCLES = 100;
  private static final int MEASURED_CYCLES = 250;
  private static final List<Pose2d> STATIONARY_POSES =
      List.of(
          pose(4.407, 7.279, -90.0),
          pose(4.407, 7.279, 90.0),
          pose(4.407, 7.279, 180.0),
          pose(4.407, 7.279, 0.0),
          pose(4.407, 0.650, 90.0),
          pose(4.407, 0.650, -90.0),
          pose(4.407, 0.650, 0.0),
          pose(4.407, 0.650, 180.0));
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
  void realPhotonSimulationDetectsAndAcceptsAllStationaryAndMovingScenarios() {
    try (VisionSimulationHarness harness =
        new VisionSimulationHarness(
            "task8-scenarios-" + UUID.randomUUID(), STATIONARY_POSES.get(0), CONFIG)) {
      for (int index = 0; index < STATIONARY_POSES.size(); index++) {
        ScenarioMetrics metrics =
            harness.runStationary(
                "stationary-" + index, STATIONARY_POSES.get(index), WARMUP_CYCLES, MEASURED_CYCLES);
        assertScenarioCovered(metrics);
        System.out.println(metrics);
      }

      ScenarioMetrics translating =
          harness.runMoving(
              "translation", pose(4.407, 7.279, -90.0), 0.65, 0.25, WARMUP_CYCLES, MEASURED_CYCLES);
      ScenarioMetrics rotating =
          harness.runMoving(
              "rotation", pose(4.407, 0.650, 90.0), 0.0, 0.40, WARMUP_CYCLES, MEASURED_CYCLES);
      Pose2d independentTruthPose = pose(4.407, 7.279, -90.0);
      Pose2d offsetEstimatorPose = pose(5.657, 7.279, -55.0);
      ScenarioMetrics independentTruth =
          harness.runStationaryWithEstimatorOffset(
              "independent-truth",
              independentTruthPose,
              offsetEstimatorPose,
              WARMUP_CYCLES,
              MEASURED_CYCLES);

      assertScenarioCovered(translating);
      assertScenarioCovered(rotating);
      assertScenarioCovered(independentTruth);
      assertAll(
          "detections and capture scoring follow truth rather than the offset estimator",
          () ->
              assertEquals(
                  1.25,
                  independentTruthPose
                      .getTranslation()
                      .getDistance(offsetEstimatorPose.getTranslation()),
                  1e-12),
          () -> assertTrue(independentTruth.meanCaptureTimeErrorMeters() < 0.25),
          () -> assertTrue(independentTruth.maxCaptureTimeErrorMeters() < 0.50));
      assertAll(
          "startup strategy stays active for moving scenarios",
          () ->
              assertTrue(
                  translating.emittedStrategyCount(PoseSolveStrategy.CONSTRAINED_SOLVEPNP) > 0),
          () ->
              assertEquals(
                  0, translating.emittedStrategyCount(PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE)),
          () ->
              assertTrue(rotating.emittedStrategyCount(PoseSolveStrategy.CONSTRAINED_SOLVEPNP) > 0),
          () ->
              assertEquals(
                  0, rotating.emittedStrategyCount(PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE)));
      System.out.println(translating);
      System.out.println(rotating);
      System.out.println(independentTruth);
    }
  }

  private static void assertScenarioCovered(ScenarioMetrics metrics) {
    assertAll(
        metrics.name(),
        () -> assertTrue(metrics.detectionCount() > 0, "camera detections"),
        () -> assertTrue(metrics.acceptedConsumerCalls() > 0, "accepted consumer calls"),
        () -> assertTrue(metrics.distinctSelectedTimestamps() > 0, "new selected timestamps"),
        () -> assertTrue(Double.isFinite(metrics.meanCaptureTimeErrorMeters())),
        () -> assertTrue(Double.isFinite(metrics.maxCaptureTimeErrorMeters())));
  }

  private static Pose2d pose(double xMeters, double yMeters, double headingDegrees) {
    return new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(headingDegrees));
  }
}
