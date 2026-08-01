package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import frc.robot.subsystems.vision.VisionStartupStrategyComparisonSupport.StrategyMetrics;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class VisionStartupStrategyComparatorTest {
  private static final StrategyMetrics LAST_VERIFIED_REFERENCE_METRICS =
      new StrategyMetrics(
          StartupStrategyOrder.REFERENCE,
          0.03880436568581087,
          0.025126729434382614,
          0,
          0.9304,
          2326,
          0.04278957838294224,
          0.2401256323063753);
  private static final StrategyMetrics LAST_VERIFIED_CONSTRAINED_SECOND_METRICS =
      new StrategyMetrics(
          StartupStrategyOrder.CONSTRAINED_SECOND,
          0.03880436568581087,
          0.025126729434382614,
          0,
          0.9304,
          2326,
          0.04279785324478572,
          0.2401256323063753);

  @Test
  void safetyGatesAcceptTheirExactBoundariesAndRejectEveryViolation() {
    StrategyMetrics exactBoundaries =
        metrics(StartupStrategyOrder.REFERENCE, 0.15, 0.25, 0, 0.30, 1, 0.10, 0.20);

    assertTrue(exactBoundaries.passesSafetyGates());
    assertFalse(
        metrics(StartupStrategyOrder.REFERENCE, Math.nextUp(0.15), 0.25, 0, 0.30, 1, 0.10, 0.20)
            .passesSafetyGates());
    assertFalse(
        metrics(StartupStrategyOrder.REFERENCE, 0.15, Math.nextUp(0.25), 0, 0.30, 1, 0.10, 0.20)
            .passesSafetyGates());
    assertFalse(
        metrics(StartupStrategyOrder.REFERENCE, 0.15, 0.25, 1, 0.30, 1, 0.10, 0.20)
            .passesSafetyGates());
    assertFalse(
        metrics(StartupStrategyOrder.REFERENCE, 0.15, 0.25, 0, Math.nextDown(0.30), 1, 0.10, 0.20)
            .passesSafetyGates());
    assertFalse(
        metrics(StartupStrategyOrder.REFERENCE, 0.15, 0.25, 0, 0.30, 0, 0.10, 0.20)
            .passesSafetyGates());
  }

  @Test
  void onlySafeOrderWins() {
    StrategyMetrics safeReference = safeMetrics(StartupStrategyOrder.REFERENCE);
    StrategyMetrics unsafeConstrained =
        metrics(StartupStrategyOrder.CONSTRAINED_SECOND, 0.16, 0.10, 0, 0.50, 10, 0.10, 0.20);
    StrategyMetrics unsafeReference =
        metrics(StartupStrategyOrder.REFERENCE, 0.16, 0.10, 0, 0.50, 10, 0.10, 0.20);
    StrategyMetrics safeConstrained = safeMetrics(StartupStrategyOrder.CONSTRAINED_SECOND);

    assertEquals(
        Optional.of(StartupStrategyOrder.REFERENCE),
        VisionStartupStrategyComparisonSupport.winner(safeReference, unsafeConstrained));
    assertEquals(
        Optional.of(StartupStrategyOrder.CONSTRAINED_SECOND),
        VisionStartupStrategyComparisonSupport.winner(unsafeReference, safeConstrained));
  }

  @Test
  void neitherSafeOrderProducesNoWinner() {
    StrategyMetrics unsafeReference =
        metrics(StartupStrategyOrder.REFERENCE, 0.16, 0.10, 0, 0.50, 10, 0.10, 0.20);
    StrategyMetrics unsafeConstrained =
        metrics(StartupStrategyOrder.CONSTRAINED_SECOND, 0.10, 0.26, 0, 0.50, 10, 0.10, 0.20);

    assertEquals(
        Optional.empty(),
        VisionStartupStrategyComparisonSupport.winner(unsafeReference, unsafeConstrained));
  }

  @Test
  void lowerMeanErrorWinsBeyondTwoCentimeterTolerance() {
    StrategyMetrics reference =
        metrics(StartupStrategyOrder.REFERENCE, 0.10, 0.10, 0, 0.50, 10, 0.079, 0.30);
    StrategyMetrics constrained =
        metrics(StartupStrategyOrder.CONSTRAINED_SECOND, 0.10, 0.10, 0, 0.50, 10, 0.10, 0.10);

    assertEquals(
        Optional.of(StartupStrategyOrder.REFERENCE),
        VisionStartupStrategyComparisonSupport.winner(reference, constrained));
    assertEquals(
        Optional.of(StartupStrategyOrder.CONSTRAINED_SECOND),
        VisionStartupStrategyComparisonSupport.winner(
            metrics(StartupStrategyOrder.REFERENCE, 0.10, 0.10, 0, 0.50, 10, 0.10, 0.10),
            metrics(
                StartupStrategyOrder.CONSTRAINED_SECOND, 0.10, 0.10, 0, 0.50, 10, 0.079, 0.30)));
  }

  @Test
  void meanErrorEqualityAdvancesToLowerWorstError() {
    StrategyMetrics reference =
        metrics(StartupStrategyOrder.REFERENCE, 0.10, 0.10, 0, 0.50, 10, 0.00, 0.20);
    StrategyMetrics constrained =
        metrics(StartupStrategyOrder.CONSTRAINED_SECOND, 0.10, 0.10, 0, 0.50, 10, 0.02, 0.10);

    assertEquals(
        0.02, constrained.meanAcceptedPoseErrorMeters() - reference.meanAcceptedPoseErrorMeters());
    assertEquals(
        Optional.of(StartupStrategyOrder.CONSTRAINED_SECOND),
        VisionStartupStrategyComparisonSupport.winner(reference, constrained));
  }

  @Test
  void lowerWorstErrorWinsBeyondFiveCentimeterTolerance() {
    StrategyMetrics reference =
        metrics(StartupStrategyOrder.REFERENCE, 0.10, 0.10, 0, 0.50, 10, 0.10, 0.20);
    StrategyMetrics constrained =
        metrics(StartupStrategyOrder.CONSTRAINED_SECOND, 0.10, 0.10, 0, 0.50, 10, 0.10, 0.251);

    assertEquals(
        Optional.of(StartupStrategyOrder.REFERENCE),
        VisionStartupStrategyComparisonSupport.winner(reference, constrained));
  }

  @Test
  void constrainedSecondWinsWhenItsWorstErrorIsLowerBeyondTolerance() {
    StrategyMetrics reference =
        metrics(StartupStrategyOrder.REFERENCE, 0.10, 0.10, 0, 0.50, 10, 0.10, 0.25);
    StrategyMetrics constrained =
        metrics(StartupStrategyOrder.CONSTRAINED_SECOND, 0.10, 0.10, 0, 0.50, 10, 0.10, 0.10);

    assertEquals(
        Optional.of(StartupStrategyOrder.CONSTRAINED_SECOND),
        VisionStartupStrategyComparisonSupport.winner(reference, constrained));
  }

  @Test
  void meanErrorPriorityOverridesOpposingWorstErrorAndCoverage() {
    StrategyMetrics reference =
        metrics(StartupStrategyOrder.REFERENCE, 0.10, 0.10, 0, 0.40, 10, 0.05, 0.30);
    StrategyMetrics constrained =
        metrics(StartupStrategyOrder.CONSTRAINED_SECOND, 0.10, 0.10, 0, 0.60, 10, 0.08, 0.20);

    assertEquals(
        Optional.of(StartupStrategyOrder.REFERENCE),
        VisionStartupStrategyComparisonSupport.winner(reference, constrained));
  }

  @Test
  void worstErrorPriorityOverridesOpposingCoverageWhenMeansTie() {
    StrategyMetrics reference =
        metrics(StartupStrategyOrder.REFERENCE, 0.10, 0.10, 0, 0.40, 10, 0.10, 0.10);
    StrategyMetrics constrained =
        metrics(StartupStrategyOrder.CONSTRAINED_SECOND, 0.10, 0.10, 0, 0.60, 10, 0.10, 0.20);

    assertEquals(
        Optional.of(StartupStrategyOrder.REFERENCE),
        VisionStartupStrategyComparisonSupport.winner(reference, constrained));
  }

  @Test
  void worstErrorEqualityAdvancesToHigherCoverage() {
    StrategyMetrics reference =
        metrics(StartupStrategyOrder.REFERENCE, 0.10, 0.10, 0, 0.50, 10, 0.10, 0.00);
    StrategyMetrics constrained =
        metrics(StartupStrategyOrder.CONSTRAINED_SECOND, 0.10, 0.10, 0, 0.551, 10, 0.10, 0.05);

    assertEquals(
        0.05,
        constrained.worstAcceptedPoseErrorMeters() - reference.worstAcceptedPoseErrorMeters());
    assertEquals(
        Optional.of(StartupStrategyOrder.CONSTRAINED_SECOND),
        VisionStartupStrategyComparisonSupport.winner(reference, constrained));
  }

  @Test
  void coverageEqualityAdvancesToConstrainedSecondTiePreference() {
    StrategyMetrics reference =
        metrics(StartupStrategyOrder.REFERENCE, 0.10, 0.10, 0, 0.55, 10, 0.10, 0.20);
    StrategyMetrics constrained =
        metrics(StartupStrategyOrder.CONSTRAINED_SECOND, 0.10, 0.10, 0, 0.50, 10, 0.10, 0.20);

    assertEquals(0.05, reference.observationCoverage() - constrained.observationCoverage(), 1e-12);
    assertEquals(
        Optional.of(StartupStrategyOrder.CONSTRAINED_SECOND),
        VisionStartupStrategyComparisonSupport.winner(reference, constrained));
  }

  @Test
  void higherCoverageWinsBeyondFivePercentTolerance() {
    StrategyMetrics reference =
        metrics(StartupStrategyOrder.REFERENCE, 0.10, 0.10, 0, 0.50, 10, 0.10, 0.20);
    StrategyMetrics constrained =
        metrics(StartupStrategyOrder.CONSTRAINED_SECOND, 0.10, 0.10, 0, 0.551, 10, 0.10, 0.20);

    assertEquals(
        Optional.of(StartupStrategyOrder.CONSTRAINED_SECOND),
        VisionStartupStrategyComparisonSupport.winner(reference, constrained));
    assertEquals(
        Optional.of(StartupStrategyOrder.REFERENCE),
        VisionStartupStrategyComparisonSupport.winner(
            metrics(StartupStrategyOrder.REFERENCE, 0.10, 0.10, 0, 0.551, 10, 0.10, 0.20),
            metrics(StartupStrategyOrder.CONSTRAINED_SECOND, 0.10, 0.10, 0, 0.50, 10, 0.10, 0.20)));
  }

  @Test
  void otherwiseConstrainedSecondWins() {
    assertEquals(
        Optional.of(StartupStrategyOrder.CONSTRAINED_SECOND),
        VisionStartupStrategyComparisonSupport.winner(
            safeMetrics(StartupStrategyOrder.REFERENCE),
            safeMetrics(StartupStrategyOrder.CONSTRAINED_SECOND)));
  }

  @Test
  void compiledDefaultMatchesWinnerFromLastVerifiedRawMetrics() {
    assertTrue(LAST_VERIFIED_REFERENCE_METRICS.passesSafetyGates());
    assertTrue(LAST_VERIFIED_CONSTRAINED_SECOND_METRICS.passesSafetyGates());
    assertEquals(
        Optional.of(VisionConstants.DEFAULT_STARTUP_STRATEGY_ORDER),
        VisionStartupStrategyComparisonSupport.winner(
            LAST_VERIFIED_REFERENCE_METRICS, LAST_VERIFIED_CONSTRAINED_SECOND_METRICS));
  }

  @Test
  void negativeMovingExcessJumpCountFailsTheExactZeroSafetyGate() {
    assertFalse(
        metrics(StartupStrategyOrder.REFERENCE, 0.10, 0.10, -1, 0.50, 10, 0.10, 0.20)
            .passesSafetyGates());
  }

  private static StrategyMetrics safeMetrics(StartupStrategyOrder order) {
    return metrics(order, 0.10, 0.10, 0, 0.50, 10, 0.10, 0.20);
  }

  private static StrategyMetrics metrics(
      StartupStrategyOrder order,
      double maximumStationaryEstimatorJumpMeters,
      double maximumMovingExcessJumpMeters,
      int movingExcessJumpOverFortyCentimetersCount,
      double observationCoverage,
      int acceptedObservationCount,
      double meanAcceptedPoseErrorMeters,
      double worstAcceptedPoseErrorMeters) {
    return new StrategyMetrics(
        order,
        maximumStationaryEstimatorJumpMeters,
        maximumMovingExcessJumpMeters,
        movingExcessJumpOverFortyCentimetersCount,
        observationCoverage,
        acceptedObservationCount,
        meanAcceptedPoseErrorMeters,
        worstAcceptedPoseErrorMeters);
  }
}
