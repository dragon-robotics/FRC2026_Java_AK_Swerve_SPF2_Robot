package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.VisionIO.PoseSolveStrategy;
import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import frc.robot.subsystems.vision.VisionRuntimeConfig.TagDistanceConfidenceMode;
import frc.robot.util.constants.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

class VisionStrategyOrderTest {
  private static final List<PoseSolveStrategy> REFERENCE =
      List.of(
          PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
          PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
          PoseSolveStrategy.LOWEST_AMBIGUITY);
  private static final List<PoseSolveStrategy> CONSTRAINED_SECOND =
      List.of(
          PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
          PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
          PoseSolveStrategy.LOWEST_AMBIGUITY);
  private static final List<PoseSolveStrategy> STATIC_ORDER =
      List.of(
          PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
          PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
          PoseSolveStrategy.LOWEST_AMBIGUITY);

  @Test
  void startupOrdersKeepTheirExactSolverPrecedence() {
    assertEquals(
        REFERENCE, VisionIOPhotonVision.startupStrategyOrder(StartupStrategyOrder.REFERENCE));
    assertEquals(
        CONSTRAINED_SECOND,
        VisionIOPhotonVision.startupStrategyOrder(StartupStrategyOrder.CONSTRAINED_SECOND));
  }

  @Test
  void hybridOrderCoversEveryMotionAndGeometryBranch() {
    assertEquals(
        List.of(
            PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
            PoseSolveStrategy.LOWEST_AMBIGUITY),
        VisionIOPhotonVision.hybridStrategyOrder(2, true, 0.0, 0.500_001));
    assertEquals(
        List.of(
            PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
            PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseSolveStrategy.LOWEST_AMBIGUITY),
        VisionIOPhotonVision.hybridStrategyOrder(1, true, 0.0, -0.500_001));
    assertEquals(
        List.of(
            PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
            PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
            PoseSolveStrategy.LOWEST_AMBIGUITY),
        VisionIOPhotonVision.hybridStrategyOrder(2, true, 0.0, 0.5));
    assertEquals(
        List.of(
            PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
            PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
            PoseSolveStrategy.LOWEST_AMBIGUITY),
        VisionIOPhotonVision.hybridStrategyOrder(2, false, 0.0, -0.5));
    assertEquals(
        List.of(
            PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
            PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
            PoseSolveStrategy.LOWEST_AMBIGUITY),
        VisionIOPhotonVision.hybridStrategyOrder(1, true, 0.500_001, 0.0));
    assertEquals(
        List.of(
            PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
            PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
            PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseSolveStrategy.LOWEST_AMBIGUITY),
        VisionIOPhotonVision.hybridStrategyOrder(1, true, 0.5, 0.0));
  }

  @Test
  void configuredSelectionStartsWithStartupOrderThenTransitionsOnlyOnce() {
    PhotonPipelineResult result = result(2, 25, 26);
    StrategyProbe explicit =
        probe(
            config(
                "HYBRID",
                List.of(PoseSolveStrategy.LOWEST_AMBIGUITY, PoseSolveStrategy.CONSTRAINED_SOLVEPNP),
                true,
                StartupStrategyOrder.REFERENCE),
            0.0,
            0.0);
    StrategyProbe hybrid =
        probe(config("HYBRID", List.of(), false, StartupStrategyOrder.REFERENCE), 0.0, 0.0);
    StrategyProbe nonHybrid =
        probe(config("STATIC", List.of(), false, StartupStrategyOrder.REFERENCE), 0.0, 0.0);
    StrategyProbe invalidExplicit =
        probe(config("HYBRID", CONSTRAINED_SECOND, true, StartupStrategyOrder.REFERENCE), 0.0, 0.0);

    assertEquals(REFERENCE, explicit.order(result));
    explicit.markVisionInitializationComplete();
    assertEquals(
        List.of(PoseSolveStrategy.LOWEST_AMBIGUITY, PoseSolveStrategy.CONSTRAINED_SOLVEPNP),
        explicit.order(result));
    explicit.markVisionInitializationComplete();
    assertEquals(
        List.of(PoseSolveStrategy.LOWEST_AMBIGUITY, PoseSolveStrategy.CONSTRAINED_SOLVEPNP),
        explicit.order(result));

    hybrid.markVisionInitializationComplete();
    assertEquals(
        List.of(
            PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
            PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
            PoseSolveStrategy.LOWEST_AMBIGUITY),
        hybrid.order(result));

    nonHybrid.markVisionInitializationComplete();
    assertEquals(STATIC_ORDER, nonHybrid.order(result));

    invalidExplicit.markVisionInitializationComplete();
    assertEquals(CONSTRAINED_SECOND, invalidExplicit.order(result));
  }

  @Test
  void hybridCoplanarityIgnoresNonpositiveFiducialIds() {
    StrategyProbe hybrid =
        probe(config("HYBRID", List.of(), false, StartupStrategyOrder.REFERENCE), 0.0, 0.0);
    hybrid.markVisionInitializationComplete();

    assertEquals(
        List.of(
            PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
            PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
            PoseSolveStrategy.LOWEST_AMBIGUITY),
        hybrid.order(result(3, -1, 26, 27)));
  }

  private static StrategyProbe probe(
      VisionRuntimeConfig config, double linearSpeed, double angularRate) {
    CameraConfig camera = new CameraConfig("test", new Transform3d(), 1.0);
    return new StrategyProbe(
        camera,
        config,
        new EmptyCameraSource(),
        new PhotonPoseEstimator(FieldConstants.APTAG_FIELD_LAYOUT, camera.robotToCamera()),
        new FixedHeadingProvider(linearSpeed, angularRate));
  }

  private static VisionRuntimeConfig config(
      String mode,
      List<PoseSolveStrategy> order,
      boolean explicit,
      StartupStrategyOrder startupOrder) {
    return new VisionRuntimeConfig(
        8.0, true, mode, order, explicit, TagDistanceConfidenceMode.ALL_TAG_AVERAGE, startupOrder);
  }

  private static PhotonPipelineResult result(long sequence, int... tagIds) {
    List<PhotonTrackedTarget> targets = new ArrayList<>();
    for (int tagId : tagIds) {
      targets.add(target(tagId));
    }
    return new PhotonPipelineResult(sequence, sequence * 1_000_000, 0, 0, targets);
  }

  private static PhotonTrackedTarget target(int tagId) {
    List<TargetCorner> corners =
        List.of(
            new TargetCorner(0.0, 0.0),
            new TargetCorner(1.0, 0.0),
            new TargetCorner(1.0, 1.0),
            new TargetCorner(0.0, 1.0));
    return new PhotonTrackedTarget(
        0.0,
        0.0,
        1.0,
        0.0,
        tagId,
        -1,
        -1.0f,
        new Transform3d(),
        new Transform3d(),
        0.1,
        corners,
        corners);
  }

  private static final class StrategyProbe extends VisionIOPhotonVision {
    StrategyProbe(
        CameraConfig camera,
        VisionRuntimeConfig config,
        CameraSource cameraSource,
        PhotonPoseEstimator estimator,
        HeadingProvider headingProvider) {
      super(camera, config, cameraSource, estimator, headingProvider);
    }

    List<PoseSolveStrategy> order(PhotonPipelineResult result) {
      return strategyOrder(result);
    }

    @Override
    Optional<EstimatedRobotPose> attemptStrategy(
        PoseSolveStrategy strategy, PhotonPipelineResult result) {
      return Optional.of(
          new EstimatedRobotPose(Pose3d.kZero, result.getTimestampSeconds(), List.of()));
    }
  }

  private static final class EmptyCameraSource implements VisionIOPhotonVision.CameraSource {
    @Override
    public String name() {
      return "test";
    }

    @Override
    public boolean isConnected() {
      return true;
    }

    @Override
    public List<PhotonPipelineResult> getAllUnreadResults() {
      return List.of();
    }

    @Override
    public Optional<
            edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N3>>
        cameraMatrix() {
      return Optional.empty();
    }

    @Override
    public Optional<
            edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N8, edu.wpi.first.math.numbers.N1>>
        distortionCoefficients() {
      return Optional.empty();
    }
  }

  private record FixedHeadingProvider(
      double linearSpeedMetersPerSecond, double angularRateRadPerSecond)
      implements VisionIOPhotonVision.HeadingProvider {
    @Override
    public Optional<Rotation2d> headingAt(double fpgaTimestampSeconds) {
      return Optional.of(Rotation2d.kZero);
    }

    @Override
    public Optional<Pose3d> seedPoseAt(double fpgaTimestampSeconds) {
      return Optional.of(Pose3d.kZero);
    }
  }
}
