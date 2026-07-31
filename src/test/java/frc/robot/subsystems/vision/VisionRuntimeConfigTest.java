package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.vision.VisionIO.PoseSolveStrategy;
import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import frc.robot.subsystems.vision.VisionRuntimeConfig.TagDistanceConfidenceMode;
import java.util.Map;
import java.util.function.Function;
import org.junit.jupiter.api.Test;

class VisionRuntimeConfigTest {
  @Test
  void defaultsUseTheApprovedRuntimeBehavior() {
    VisionRuntimeConfig config = parse(Map.of(), StartupStrategyOrder.CONSTRAINED_SECOND);

    assertAll(
        () -> assertEquals(8.0, config.maxAbsTiltDegrees(), 1e-12),
        () -> assertTrue(config.applyCoplanarPenalty()),
        () -> assertEquals("HYBRID", config.strategyMode()),
        () -> assertEquals(java.util.List.of(), config.configuredStrategyOrder()),
        () -> assertFalse(config.explicitStrategyOrder()),
        () ->
            assertEquals(
                TagDistanceConfidenceMode.ALL_TAG_AVERAGE, config.tagDistanceConfidenceMode()),
        () -> assertEquals(StartupStrategyOrder.CONSTRAINED_SECOND, config.startupStrategyOrder()));
  }

  @Test
  void validValuesNormalizeCaseAndWhitespace() {
    VisionRuntimeConfig config =
        parse(
            Map.of(
                "vision.maxAbsTiltDeg", " 12.5 ",
                "vision.applyCoplanarPenalty", " FALSE ",
                "vision.photon.strategyMode", " static ",
                "vision.tagDistanceConfidenceMode", " max_tag_distance "),
            StartupStrategyOrder.REFERENCE);

    assertAll(
        () -> assertEquals(12.5, config.maxAbsTiltDegrees(), 1e-12),
        () -> assertFalse(config.applyCoplanarPenalty()),
        () -> assertEquals("STATIC", config.strategyMode()),
        () ->
            assertEquals(
                TagDistanceConfidenceMode.MAX_TAG_DISTANCE, config.tagDistanceConfidenceMode()),
        () -> assertEquals(StartupStrategyOrder.REFERENCE, config.startupStrategyOrder()));
  }

  @Test
  void explicitStrategyOrderKeepsSupportedTokensInTheirGivenOrder() {
    VisionRuntimeConfig config =
        parse(
            Map.of(
                "vision.photon.strategyOrder",
                " lowest_ambiguity, unsupported, constrained_solvepnp "),
            StartupStrategyOrder.CONSTRAINED_SECOND);

    assertAll(
        () -> assertTrue(config.explicitStrategyOrder()),
        () ->
            assertEquals(
                java.util.List.of(
                    PoseSolveStrategy.LOWEST_AMBIGUITY, PoseSolveStrategy.CONSTRAINED_SOLVEPNP),
                config.configuredStrategyOrder()));
  }

  @Test
  void allInvalidExplicitStrategyTokensUseTheCompleteFallbackChain() {
    VisionRuntimeConfig config =
        parse(
            Map.of("vision.photon.strategyOrder", "unsupported, still-not-a-strategy"),
            StartupStrategyOrder.CONSTRAINED_SECOND);

    assertAll(
        () -> assertTrue(config.explicitStrategyOrder()),
        () ->
            assertEquals(
                java.util.List.of(
                    PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
                    PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    PoseSolveStrategy.LOWEST_AMBIGUITY),
                config.configuredStrategyOrder()));
  }

  @Test
  void blankStrategyOrderDoesNotEnableAnExplicitOrder() {
    VisionRuntimeConfig config =
        parse(
            Map.of("vision.photon.strategyOrder", "   "), StartupStrategyOrder.CONSTRAINED_SECOND);

    assertAll(
        () -> assertFalse(config.explicitStrategyOrder()),
        () -> assertEquals(java.util.List.of(), config.configuredStrategyOrder()));
  }

  @Test
  void invalidTiltBooleanAndDistanceModeFallBackToDefaults() {
    VisionRuntimeConfig config =
        parse(
            Map.of(
                "vision.maxAbsTiltDeg", "NaN",
                "vision.applyCoplanarPenalty", "not-a-boolean",
                "vision.tagDistanceConfidenceMode", "closest-tag"),
            StartupStrategyOrder.CONSTRAINED_SECOND);

    assertAll(
        () -> assertEquals(8.0, config.maxAbsTiltDegrees(), 1e-12),
        () -> assertTrue(config.applyCoplanarPenalty()),
        () ->
            assertEquals(
                TagDistanceConfidenceMode.ALL_TAG_AVERAGE, config.tagDistanceConfidenceMode()));
  }

  @Test
  void startupStrategyOrderIsProvidedByDirectInjection() {
    VisionRuntimeConfig config = parse(Map.of(), StartupStrategyOrder.REFERENCE);

    assertEquals(StartupStrategyOrder.REFERENCE, config.startupStrategyOrder());
  }

  private static VisionRuntimeConfig parse(
      Map<String, String> properties, StartupStrategyOrder startupStrategyOrder) {
    Function<String, String> lookup = properties::get;
    return VisionRuntimeConfig.parse(lookup, startupStrategyOrder);
  }
}
