package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionIO.PoseSolveStrategy;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Function;

/** Immutable, startup-captured runtime settings for the vision subsystem. */
public record VisionRuntimeConfig(
    double maxAbsTiltDegrees,
    boolean applyCoplanarPenalty,
    String strategyMode,
    List<PoseSolveStrategy> configuredStrategyOrder,
    boolean explicitStrategyOrder,
    TagDistanceConfidenceMode tagDistanceConfidenceMode,
    StartupStrategyOrder startupStrategyOrder) {
  private static final double DEFAULT_MAX_ABS_TILT_DEGREES = 8.0;
  private static final boolean DEFAULT_APPLY_COPLANAR_PENALTY = true;
  private static final String DEFAULT_STRATEGY_MODE = "HYBRID";
  private static final List<PoseSolveStrategy> FALLBACK_STRATEGY_ORDER =
      List.of(
          PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
          PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
          PoseSolveStrategy.LOWEST_AMBIGUITY);

  public enum StartupStrategyOrder {
    REFERENCE,
    CONSTRAINED_SECOND
  }

  public enum TagDistanceConfidenceMode {
    ALL_TAG_AVERAGE,
    MAX_TAG_DISTANCE
  }

  public VisionRuntimeConfig {
    configuredStrategyOrder = List.copyOf(configuredStrategyOrder);
  }

  public static VisionRuntimeConfig fromSystemProperties() {
    return parse(System::getProperty, VisionConstants.DEFAULT_STARTUP_STRATEGY_ORDER);
  }

  static VisionRuntimeConfig parse(
      Function<String, String> propertyLookup, StartupStrategyOrder startupOrder) {
    String strategyOrderProperty = propertyLookup.apply("vision.photon.strategyOrder");
    boolean explicitStrategyOrder =
        strategyOrderProperty != null && !strategyOrderProperty.trim().isEmpty();
    List<PoseSolveStrategy> configuredStrategyOrder =
        explicitStrategyOrder ? parseStrategyOrder(strategyOrderProperty) : List.of();

    return new VisionRuntimeConfig(
        parsePositiveFiniteDouble(
            propertyLookup.apply("vision.maxAbsTiltDeg"), DEFAULT_MAX_ABS_TILT_DEGREES),
        parseBoolean(
            propertyLookup.apply("vision.applyCoplanarPenalty"), DEFAULT_APPLY_COPLANAR_PENALTY),
        parseStrategyMode(propertyLookup.apply("vision.photon.strategyMode")),
        configuredStrategyOrder,
        explicitStrategyOrder,
        parseTagDistanceConfidenceMode(propertyLookup.apply("vision.tagDistanceConfidenceMode")),
        startupOrder);
  }

  private static double parsePositiveFiniteDouble(String value, double defaultValue) {
    if (value == null) {
      return defaultValue;
    }
    try {
      double parsed = Double.parseDouble(value.trim());
      return Double.isFinite(parsed) && parsed > 0.0 ? parsed : defaultValue;
    } catch (NumberFormatException exception) {
      return defaultValue;
    }
  }

  private static boolean parseBoolean(String value, boolean defaultValue) {
    if (value == null) {
      return defaultValue;
    }
    String normalized = value.trim().toLowerCase(Locale.ROOT);
    if (normalized.equals("true")) {
      return true;
    }
    if (normalized.equals("false")) {
      return false;
    }
    return defaultValue;
  }

  private static String parseStrategyMode(String value) {
    if (value == null || value.trim().isEmpty()) {
      return DEFAULT_STRATEGY_MODE;
    }
    return value.trim().toUpperCase(Locale.ROOT);
  }

  private static List<PoseSolveStrategy> parseStrategyOrder(String value) {
    List<PoseSolveStrategy> strategies = new ArrayList<>();
    for (String token : value.split(",")) {
      try {
        PoseSolveStrategy strategy =
            PoseSolveStrategy.valueOf(token.trim().toUpperCase(Locale.ROOT));
        if (strategy != PoseSolveStrategy.UNKNOWN) {
          strategies.add(strategy);
        }
      } catch (IllegalArgumentException ignored) {
      }
    }
    return strategies.isEmpty() ? FALLBACK_STRATEGY_ORDER : List.copyOf(strategies);
  }

  private static TagDistanceConfidenceMode parseTagDistanceConfidenceMode(String value) {
    if (value == null) {
      return TagDistanceConfidenceMode.ALL_TAG_AVERAGE;
    }
    try {
      return TagDistanceConfidenceMode.valueOf(value.trim().toUpperCase(Locale.ROOT));
    } catch (IllegalArgumentException exception) {
      return TagDistanceConfidenceMode.ALL_TAG_AVERAGE;
    }
  }
}
