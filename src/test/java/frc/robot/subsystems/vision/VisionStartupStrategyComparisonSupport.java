package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import java.math.BigDecimal;
import java.util.Optional;

/** Pure comparison support shared by isolated measurement and normal regression tests. */
public final class VisionStartupStrategyComparisonSupport {
  private static final double MEAN_ERROR_TOLERANCE_METERS = 0.02;
  private static final double WORST_ERROR_TOLERANCE_METERS = 0.05;
  private static final double COVERAGE_TOLERANCE = 0.05;

  private VisionStartupStrategyComparisonSupport() {}

  /** Selects the safe winner using the approved deterministic metric priority. */
  public static Optional<StartupStrategyOrder> winner(
      StrategyMetrics reference, StrategyMetrics constrainedSecond) {
    boolean referenceSafe = reference.passesSafetyGates();
    boolean constrainedSecondSafe = constrainedSecond.passesSafetyGates();

    if (referenceSafe && !constrainedSecondSafe) {
      return Optional.of(StartupStrategyOrder.REFERENCE);
    }
    if (!referenceSafe && constrainedSecondSafe) {
      return Optional.of(StartupStrategyOrder.CONSTRAINED_SECOND);
    }
    if (!referenceSafe) {
      return Optional.empty();
    }

    if (differsByMoreThan(
        reference.meanAcceptedPoseErrorMeters(),
        constrainedSecond.meanAcceptedPoseErrorMeters(),
        MEAN_ERROR_TOLERANCE_METERS)) {
      return Optional.of(
          reference.meanAcceptedPoseErrorMeters() < constrainedSecond.meanAcceptedPoseErrorMeters()
              ? StartupStrategyOrder.REFERENCE
              : StartupStrategyOrder.CONSTRAINED_SECOND);
    }
    if (differsByMoreThan(
        reference.worstAcceptedPoseErrorMeters(),
        constrainedSecond.worstAcceptedPoseErrorMeters(),
        WORST_ERROR_TOLERANCE_METERS)) {
      return Optional.of(
          reference.worstAcceptedPoseErrorMeters()
                  < constrainedSecond.worstAcceptedPoseErrorMeters()
              ? StartupStrategyOrder.REFERENCE
              : StartupStrategyOrder.CONSTRAINED_SECOND);
    }
    if (differsByMoreThan(
        reference.observationCoverage(),
        constrainedSecond.observationCoverage(),
        COVERAGE_TOLERANCE)) {
      return Optional.of(
          reference.observationCoverage() > constrainedSecond.observationCoverage()
              ? StartupStrategyOrder.REFERENCE
              : StartupStrategyOrder.CONSTRAINED_SECOND);
    }
    return Optional.of(StartupStrategyOrder.CONSTRAINED_SECOND);
  }

  private static boolean differsByMoreThan(double first, double second, double tolerance) {
    return BigDecimal.valueOf(first)
            .subtract(BigDecimal.valueOf(second))
            .abs()
            .compareTo(BigDecimal.valueOf(tolerance))
        > 0;
  }

  /** Complete raw measurements for one startup order. */
  public record StrategyMetrics(
      StartupStrategyOrder order,
      double maximumStationaryEstimatorJumpMeters,
      double maximumMovingExcessJumpMeters,
      int movingExcessJumpOverFortyCentimetersCount,
      double observationCoverage,
      int acceptedObservationCount,
      double meanAcceptedPoseErrorMeters,
      double worstAcceptedPoseErrorMeters) {
    /** Returns whether every exact safety gate passes. */
    public boolean passesSafetyGates() {
      return maximumStationaryEstimatorJumpMeters <= 0.15
          && maximumMovingExcessJumpMeters <= 0.25
          && movingExcessJumpOverFortyCentimetersCount == 0
          && observationCoverage >= 0.30
          && acceptedObservationCount > 0;
    }
  }
}
