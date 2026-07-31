package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.PoseSolveStrategy;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import frc.robot.subsystems.vision.VisionRuntimeConfig.TagDistanceConfidenceMode;
import frc.robot.util.constants.FieldConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** PhotonVision camera IO that decodes every unread frame and runs the configured pose solvers. */
public class VisionIOPhotonVision implements VisionIO {
  private static final List<PoseSolveStrategy> REFERENCE_STARTUP_ORDER =
      List.of(
          PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
          PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
          PoseSolveStrategy.LOWEST_AMBIGUITY);
  private static final List<PoseSolveStrategy> CONSTRAINED_SECOND_STARTUP_ORDER =
      List.of(
          PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
          PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
          PoseSolveStrategy.LOWEST_AMBIGUITY);
  private static final List<PoseSolveStrategy> NON_HYBRID_ORDER =
      List.of(
          PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
          PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
          PoseSolveStrategy.LOWEST_AMBIGUITY);
  private static final PoseObservation[] NO_POSE_OBSERVATIONS = new PoseObservation[0];
  private static final int[] NO_TAG_IDS = new int[0];

  interface CameraSource {
    String name();

    boolean isConnected();

    List<PhotonPipelineResult> getAllUnreadResults();

    Optional<Matrix<N3, N3>> cameraMatrix();

    Optional<Matrix<N8, N1>> distortionCoefficients();
  }

  /** Supplies timestamped drivetrain state to the heading-assisted pose solvers. */
  public interface HeadingProvider {
    Optional<Rotation2d> headingAt(double fpgaTimestampSeconds);

    Optional<Pose3d> seedPoseAt(double fpgaTimestampSeconds);

    double angularRateRadPerSecond();

    double linearSpeedMetersPerSecond();
  }

  private final VisionRuntimeConfig runtimeConfig;
  private final CameraSource cameraSource;
  protected final PhotonPoseEstimator poseEstimator;
  private final HeadingProvider headingProvider;
  private boolean visionInitializationComplete;

  /** Creates an IO adapter backed by the real PhotonVision camera named in {@code cameraConfig}. */
  public VisionIOPhotonVision(
      CameraConfig cameraConfig,
      VisionRuntimeConfig runtimeConfig,
      HeadingProvider headingProvider) {
    this(
        cameraConfig,
        runtimeConfig,
        new PhotonCameraSource(new PhotonCamera(cameraConfig.name())),
        new PhotonPoseEstimator(FieldConstants.APTAG_FIELD_LAYOUT, cameraConfig.robotToCamera()),
        headingProvider);
  }

  VisionIOPhotonVision(
      CameraConfig cameraConfig,
      VisionRuntimeConfig runtimeConfig,
      CameraSource cameraSource,
      PhotonPoseEstimator poseEstimator,
      HeadingProvider headingProvider) {
    this.runtimeConfig = runtimeConfig;
    this.cameraSource = cameraSource;
    this.poseEstimator = poseEstimator;
    this.headingProvider = headingProvider;
  }

  @Override
  public String getCameraName() {
    return cameraSource.name();
  }

  @Override
  public void markVisionInitializationComplete() {
    if (!visionInitializationComplete) {
      visionInitializationComplete = true;
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraName = cameraSource.name();
    inputs.connected = cameraSource.isConnected();
    inputs.latestTargetObservation = TargetObservation.NONE;
    inputs.setPoseObservations(NO_POSE_OBSERVATIONS);
    inputs.setTagIds(NO_TAG_IDS);

    List<PoseObservation> observations = new ArrayList<>();
    List<Integer> cameraTagIds = new ArrayList<>();
    for (PhotonPipelineResult result : cameraSource.getAllUnreadResults()) {
      processResult(result, inputs, observations, cameraTagIds);
    }

    inputs.setPoseObservations(observations.toArray(NO_POSE_OBSERVATIONS));
    inputs.setTagIds(cameraTagIds.stream().mapToInt(Integer::intValue).toArray());
  }

  private void processResult(
      PhotonPipelineResult result,
      VisionIOInputs inputs,
      List<PoseObservation> observations,
      List<Integer> cameraTagIds) {
    if (result == null || !result.hasTargets()) {
      inputs.latestTargetObservation = TargetObservation.NONE;
      return;
    }

    PhotonTrackedTarget bestTarget = result.getBestTarget();
    if (bestTarget == null) {
      inputs.latestTargetObservation = TargetObservation.NONE;
      return;
    }
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(bestTarget.getYaw()),
            Rotation2d.fromDegrees(bestTarget.getPitch()));

    List<PhotonTrackedTarget> targets = result.getTargets();
    if (targets.isEmpty() || allTargetsBeyondMaximumRange(targets)) {
      return;
    }

    for (PoseSolveStrategy strategy : strategyOrder(result)) {
      Optional<EstimatedRobotPose> estimate = attemptStrategy(strategy, result);
      if (estimate.isEmpty()) {
        continue;
      }

      Optional<PoseObservation> observation = toPoseObservation(estimate.orElseThrow(), strategy);
      if (observation.isPresent()) {
        PoseObservation emitted = observation.orElseThrow();
        observations.add(emitted);
        addCameraTagIds(cameraTagIds, emitted.tagIds());
      }
      return;
    }
  }

  List<PoseSolveStrategy> strategyOrder(PhotonPipelineResult result) {
    if (!visionInitializationComplete) {
      return startupStrategyOrder(runtimeConfig.startupStrategyOrder());
    }
    if (runtimeConfig.explicitStrategyOrder()) {
      return runtimeConfig.configuredStrategyOrder();
    }
    if (!"HYBRID".equalsIgnoreCase(runtimeConfig.strategyMode())) {
      return NON_HYBRID_ORDER;
    }

    List<PhotonTrackedTarget> targets = result.getTargets();
    int[] positiveTagIds = positiveTagIds(targets);
    boolean coplanarTargetSet =
        VisionGeometry.areTagsCoplanar(FieldConstants.APTAG_FIELD_LAYOUT, positiveTagIds);
    double linearSpeed =
        headingProvider == null ? 0.0 : headingProvider.linearSpeedMetersPerSecond();
    double angularRate = headingProvider == null ? 0.0 : headingProvider.angularRateRadPerSecond();
    return hybridStrategyOrder(targets.size(), coplanarTargetSet, linearSpeed, angularRate);
  }

  static List<PoseSolveStrategy> startupStrategyOrder(StartupStrategyOrder startupOrder) {
    return startupOrder == StartupStrategyOrder.REFERENCE
        ? REFERENCE_STARTUP_ORDER
        : CONSTRAINED_SECOND_STARTUP_ORDER;
  }

  static List<PoseSolveStrategy> hybridStrategyOrder(
      int visibleTargetCount,
      boolean coplanarTargetSet,
      double linearSpeedMetersPerSecond,
      double angularRateRadPerSecond) {
    if (Math.abs(angularRateRadPerSecond)
        > VisionConstants.CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC) {
      if (visibleTargetCount >= 2) {
        return List.of(
            PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
            PoseSolveStrategy.LOWEST_AMBIGUITY);
      }
      return List.of(
          PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
          PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PoseSolveStrategy.LOWEST_AMBIGUITY);
    }

    if (visibleTargetCount >= 2) {
      if (coplanarTargetSet) {
        return List.of(
            PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
            PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
            PoseSolveStrategy.LOWEST_AMBIGUITY);
      }
      return List.of(
          PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
          PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
          PoseSolveStrategy.LOWEST_AMBIGUITY);
    }

    if (linearSpeedMetersPerSecond > VisionConstants.HYBRID_TRANSLATION_SPEED_THRESHOLD_MPS) {
      return List.of(
          PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
          PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
          PoseSolveStrategy.LOWEST_AMBIGUITY);
    }
    return List.of(
        PoseSolveStrategy.PNP_DISTANCE_TRIG_SOLVE,
        PoseSolveStrategy.CONSTRAINED_SOLVEPNP,
        PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PoseSolveStrategy.LOWEST_AMBIGUITY);
  }

  Optional<EstimatedRobotPose> attemptStrategy(
      PoseSolveStrategy strategy, PhotonPipelineResult result) {
    return switch (strategy) {
      case MULTI_TAG_PNP_ON_COPROCESSOR -> poseEstimator.estimateCoprocMultiTagPose(result);
      case LOWEST_AMBIGUITY -> poseEstimator.estimateLowestAmbiguityPose(result);
      case PNP_DISTANCE_TRIG_SOLVE -> attemptTrigStrategy(result);
      case CONSTRAINED_SOLVEPNP -> attemptConstrainedStrategy(result);
      case UNKNOWN -> Optional.empty();
    };
  }

  private Optional<EstimatedRobotPose> attemptTrigStrategy(PhotonPipelineResult result) {
    if (headingProvider == null
        || Math.abs(headingProvider.angularRateRadPerSecond())
            > VisionConstants.TRIG_MAX_ANGULAR_RATE_RAD_PER_SEC) {
      return Optional.empty();
    }

    double timestampSeconds = result.getTimestampSeconds();
    Optional<Rotation2d> heading = headingProvider.headingAt(timestampSeconds);
    if (heading.isEmpty()) {
      return Optional.empty();
    }
    poseEstimator.addHeadingData(timestampSeconds, heading.orElseThrow());
    return poseEstimator.estimatePnpDistanceTrigSolvePose(result);
  }

  private Optional<EstimatedRobotPose> attemptConstrainedStrategy(PhotonPipelineResult result) {
    if (headingProvider == null
        || Math.abs(headingProvider.angularRateRadPerSecond())
            > VisionConstants.CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC) {
      return Optional.empty();
    }

    double timestampSeconds = result.getTimestampSeconds();
    Optional<Rotation2d> heading = headingProvider.headingAt(timestampSeconds);
    if (heading.isEmpty()) {
      return Optional.empty();
    }

    Optional<Pose3d> seedPose =
        poseEstimator
            .estimateLowestAmbiguityPose(result)
            .map(estimatedRobotPose -> estimatedRobotPose.estimatedPose);
    if (seedPose.isEmpty()) {
      seedPose = headingProvider.seedPoseAt(timestampSeconds);
    }
    if (seedPose.isEmpty()) {
      return Optional.empty();
    }

    Optional<Matrix<N3, N3>> cameraMatrix = cameraSource.cameraMatrix();
    Optional<Matrix<N8, N1>> distortionCoefficients = cameraSource.distortionCoefficients();
    if (cameraMatrix.isEmpty() || distortionCoefficients.isEmpty()) {
      return Optional.empty();
    }

    poseEstimator.addHeadingData(timestampSeconds, heading.orElseThrow());
    return poseEstimator.estimateConstrainedSolvepnpPose(
        result,
        cameraMatrix.orElseThrow(),
        distortionCoefficients.orElseThrow(),
        seedPose.orElseThrow(),
        false,
        VisionConstants.CONSTRAINED_HEADING_SCALE_FACTOR);
  }

  private Optional<PoseObservation> toPoseObservation(
      EstimatedRobotPose estimatedPose, PoseSolveStrategy strategy) {
    List<PhotonTrackedTarget> targets = estimatedPose.targetsUsed;
    int[] observedTagIds = new int[targets.size()];
    int observedTagCount = 0;
    int distanceSampleCount = 0;
    double totalDistanceMeters = 0.0;
    double maximumDistanceMeters = 0.0;
    double totalAmbiguity = 0.0;

    for (PhotonTrackedTarget target : targets) {
      int tagId = target.getFiducialId();
      if (tagId <= 0) {
        continue;
      }

      observedTagIds[observedTagCount++] = tagId;
      totalAmbiguity += Math.max(0.0, target.getPoseAmbiguity());
      if (target.getBestCameraToTarget() != null) {
        double distanceMeters = target.getBestCameraToTarget().getTranslation().getNorm();
        totalDistanceMeters += distanceMeters;
        distanceSampleCount++;
        maximumDistanceMeters = Math.max(maximumDistanceMeters, distanceMeters);
      }
    }

    if (observedTagCount == 0) {
      return Optional.empty();
    }
    observedTagIds = Arrays.copyOf(observedTagIds, observedTagCount);
    double confidenceDistance =
        confidenceDistance(
            runtimeConfig.tagDistanceConfidenceMode(),
            totalDistanceMeters,
            distanceSampleCount,
            maximumDistanceMeters);
    PoseObservationType type =
        strategy == PoseSolveStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
            ? PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR
            : PoseObservationType.PHOTONVISION;
    return Optional.of(
        new PoseObservation(
            estimatedPose.timestampSeconds,
            estimatedPose.estimatedPose,
            totalAmbiguity / observedTagCount,
            observedTagCount,
            confidenceDistance,
            type,
            strategy,
            observedTagIds));
  }

  private static double confidenceDistance(
      TagDistanceConfidenceMode mode,
      double totalDistanceMeters,
      int distanceSampleCount,
      double maximumDistanceMeters) {
    if (distanceSampleCount == 0) {
      return Double.POSITIVE_INFINITY;
    }
    return switch (mode) {
      case ALL_TAG_AVERAGE -> totalDistanceMeters / distanceSampleCount;
      case MAX_TAG_DISTANCE -> maximumDistanceMeters;
    };
  }

  private static boolean allTargetsBeyondMaximumRange(List<PhotonTrackedTarget> targets) {
    for (PhotonTrackedTarget target : targets) {
      if (target.getBestCameraToTarget() != null
          && target.getBestCameraToTarget().getTranslation().getNorm()
              <= VisionConstants.MAX_TARGET_PREFILTER_DISTANCE_METERS) {
        return false;
      }
    }
    return true;
  }

  private static int[] positiveTagIds(List<PhotonTrackedTarget> targets) {
    int[] tagIds = new int[targets.size()];
    int count = 0;
    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() > 0) {
        tagIds[count++] = target.getFiducialId();
      }
    }
    return Arrays.copyOf(tagIds, count);
  }

  private static void addCameraTagIds(List<Integer> cameraTagIds, int[] observationTagIds) {
    for (int tagId : observationTagIds) {
      if (!cameraTagIds.contains(tagId)) {
        cameraTagIds.add(tagId);
      }
    }
  }

  private record PhotonCameraSource(PhotonCamera camera) implements CameraSource {
    @Override
    public String name() {
      return camera.getName();
    }

    @Override
    public boolean isConnected() {
      return camera.isConnected();
    }

    @Override
    public List<PhotonPipelineResult> getAllUnreadResults() {
      return camera.getAllUnreadResults();
    }

    @Override
    public Optional<Matrix<N3, N3>> cameraMatrix() {
      return camera.getCameraMatrix();
    }

    @Override
    public Optional<Matrix<N8, N1>> distortionCoefficients() {
      return camera.getDistCoeffs();
    }
  }
}
