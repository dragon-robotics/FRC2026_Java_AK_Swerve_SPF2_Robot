package frc.robot.subsystems.vision.sim;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.vision.VisionRuntimeConfig;
import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import frc.robot.subsystems.vision.VisionRuntimeConfig.TagDistanceConfidenceMode;
import frc.robot.subsystems.vision.VisionSimulationHarness;
import frc.robot.subsystems.vision.VisionStartupStrategyComparisonSupport;
import frc.robot.subsystems.vision.VisionStartupStrategyComparisonSupport.StrategyMetrics;
import java.util.List;
import java.util.Optional;
import java.util.UUID;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonCamera;

@Tag("vision-sim")
class VisionStartupStrategyComparisonTest {
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
  void identicalScenariosSelectOneSafeDeterministicStartupOrder() {
    StrategyMetrics reference = runOrder(StartupStrategyOrder.REFERENCE);
    StrategyMetrics constrainedSecond = runOrder(StartupStrategyOrder.CONSTRAINED_SECOND);

    System.out.println("REFERENCE_METRICS = " + reference);
    System.out.println("CONSTRAINED_SECOND_METRICS = " + constrainedSecond);

    Optional<StartupStrategyOrder> winner =
        VisionStartupStrategyComparisonSupport.winner(reference, constrainedSecond);
    String completeMetrics = reference + System.lineSeparator() + constrainedSecond;
    assertAll(
        () -> assertEquals(StartupStrategyOrder.REFERENCE, reference.order()),
        () -> assertEquals(StartupStrategyOrder.CONSTRAINED_SECOND, constrainedSecond.order()),
        () ->
            assertTrue(
                reference.passesSafetyGates() || constrainedSecond.passesSafetyGates(),
                "Neither startup order passed the fixed safety gates:\n" + completeMetrics),
        () -> assertTrue(winner.isPresent(), "No safe winner:\n" + completeMetrics),
        () ->
            assertTrue(
                Math.min(
                        reference.maximumStationaryEstimatorJumpMeters(),
                        constrainedSecond.maximumStationaryEstimatorJumpMeters())
                    > 0.0,
                "A stationary estimator never responded to accepted measurements"),
        () ->
            assertTrue(
                Math.min(
                        reference.maximumMovingExcessJumpMeters(),
                        constrainedSecond.maximumMovingExcessJumpMeters())
                    > 0.0,
                "A moving estimator excess jump was a tautological zero"));
  }

  private static StrategyMetrics runOrder(StartupStrategyOrder order) {
    VisionRuntimeConfig config =
        new VisionRuntimeConfig(
            8.0,
            true,
            "HYBRID",
            List.of(),
            false,
            TagDistanceConfidenceMode.ALL_TAG_AVERAGE,
            order);
    try (VisionSimulationHarness harness =
        new VisionSimulationHarness(
            "task9-comparison-" + order + "-" + UUID.randomUUID(),
            STATIONARY_POSES.get(0),
            config)) {
      return harness.runStartupStrategyComparison(STATIONARY_POSES, WARMUP_CYCLES, MEASURED_CYCLES);
    }
  }

  private static Pose2d pose(double xMeters, double yMeters, double headingDegrees) {
    return new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(headingDegrees));
  }
}
