package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.vision.VisionIO.PoseSolveStrategy;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicLong;

/** Deterministic real-Photon test harness with separate truth and estimator state. */
public final class VisionSimulationHarness implements AutoCloseable {
  private static final double LOOP_PERIOD_SECONDS = 0.020;
  private static final double HISTORY_SECONDS = 2.0;

  private final TimeInterpolatableBuffer<Pose2d> truthHistory =
      TimeInterpolatableBuffer.createBuffer(HISTORY_SECONDS);
  private final TimeInterpolatableBuffer<Pose2d> estimatorHistory =
      TimeInterpolatableBuffer.createBuffer(HISTORY_SECONDS);
  private final VisionSimulation owner;
  private final CountingVisionIO[] cameraIO;
  private final Vision vision;
  private final HeadingProvider headingProvider = new HeadingProvider();
  private final Set<Double> selectedTimestamps = new HashSet<>();
  private final EnumMap<PoseSolveStrategy, Integer> emittedStrategyCounts =
      new EnumMap<>(PoseSolveStrategy.class);

  private Pose2d truthPose;
  private Pose2d estimatorPose;
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private long firstCameraReadOwnerUpdateCount = -1;
  private boolean measuring;
  private int detectionCount;
  private int acceptedConsumerCalls;
  private double captureErrorSumMeters;
  private double maxCaptureErrorMeters = Double.NEGATIVE_INFINITY;

  public VisionSimulationHarness(
      String instanceName, Pose2d initialTruth, VisionRuntimeConfig runtimeConfig) {
    truthPose = initialTruth;
    estimatorPose = new Pose2d(initialTruth.getTranslation(), initialTruth.getRotation());
    owner = new VisionSimulation(instanceName, () -> truthPose);
    cameraIO = new CountingVisionIO[VisionConstants.CAMERAS.size()];
    for (int index = 0; index < VisionConstants.CAMERAS.size(); index++) {
      cameraIO[index] =
          new CountingVisionIO(
              new VisionIOPhotonVisionSim(
                  VisionConstants.CAMERAS.get(index), runtimeConfig, headingProvider, owner));
    }

    VisionDriveBindings bindings =
        new VisionDriveBindings(
            this::recordMeasurement,
            pose -> estimatorPose = pose,
            () -> estimatorPose,
            estimatorHistory::getSample,
            () -> chassisSpeeds,
            () -> 0.0,
            () -> 0.0);
    vision =
        new Vision(
            bindings,
            runtimeConfig,
            () -> false,
            Timer::getFPGATimestamp,
            owner::update,
            false,
            cameraIO);
    resetScenario(initialTruth, initialTruth, 0.0, 0.0);
  }

  /** Runs one Vision cycle against existing adapters and records owner-hook/read ordering. */
  public static HookOrderMetrics runHookCycle(
      VisionSimulation owner,
      Pose2d currentPose,
      VisionRuntimeConfig runtimeConfig,
      VisionIO... cameraIO) {
    AtomicLong firstCameraRead = new AtomicLong(-1L);
    VisionIO[] probes = new VisionIO[cameraIO.length];
    for (int index = 0; index < cameraIO.length; index++) {
      VisionIO delegate = cameraIO[index];
      probes[index] =
          new VisionIO() {
            @Override
            public String getCameraName() {
              return delegate.getCameraName();
            }

            @Override
            public void updateInputs(VisionIOInputs inputs) {
              firstCameraRead.compareAndSet(-1L, owner.updateCount());
              delegate.updateInputs(inputs);
            }

            @Override
            public void markVisionInitializationComplete() {
              delegate.markVisionInitializationComplete();
            }
          };
    }
    VisionDriveBindings bindings =
        new VisionDriveBindings(
            (pose, timestampSeconds, standardDeviations) -> {},
            pose -> {},
            () -> currentPose,
            timestampSeconds -> Optional.of(currentPose),
            ChassisSpeeds::new,
            () -> 0.0,
            () -> 0.0);
    new Vision(
            bindings,
            runtimeConfig,
            () -> false,
            Timer::getFPGATimestamp,
            owner::update,
            true,
            probes)
        .periodic();
    return new HookOrderMetrics(owner.updateCount(), firstCameraRead.get());
  }

  public static boolean usesExactOwnedCamera(
      VisionSimulation owner, VisionIOPhotonVisionSim adapter) {
    return adapter.usesExactOwnedCamera(owner);
  }

  /** Runs one normal Vision loop after advancing FPGA simulation time by 20 ms. */
  public void periodic() {
    advanceState();
    vision.periodic();
  }

  public ScenarioMetrics runStationary(
      String name, Pose2d pose, int warmupCycles, int measuredCycles) {
    return runScenario(name, pose, 0.0, 0.0, warmupCycles, measuredCycles);
  }

  public ScenarioMetrics runMoving(
      String name,
      Pose2d start,
      double linearVelocityMetersPerSecond,
      double angularVelocityRadPerSecond,
      int warmupCycles,
      int measuredCycles) {
    return runScenario(
        name,
        start,
        linearVelocityMetersPerSecond,
        angularVelocityRadPerSecond,
        warmupCycles,
        measuredCycles);
  }

  public ScenarioMetrics runStationaryWithEstimatorOffset(
      String name, Pose2d truthPose, Pose2d estimatorPose, int warmupCycles, int measuredCycles) {
    return runScenario(name, truthPose, estimatorPose, 0.0, 0.0, warmupCycles, measuredCycles);
  }

  public long ownerUpdateCount() {
    return owner.updateCount();
  }

  public long firstCameraReadOwnerUpdateCount() {
    return firstCameraReadOwnerUpdateCount;
  }

  @Override
  public void close() {
    owner.close();
    truthHistory.clear();
    estimatorHistory.clear();
    selectedTimestamps.clear();
  }

  private ScenarioMetrics runScenario(
      String name,
      Pose2d start,
      double linearVelocityMetersPerSecond,
      double angularVelocityRadPerSecond,
      int warmupCycles,
      int measuredCycles) {
    return runScenario(
        name,
        start,
        start,
        linearVelocityMetersPerSecond,
        angularVelocityRadPerSecond,
        warmupCycles,
        measuredCycles);
  }

  private ScenarioMetrics runScenario(
      String name,
      Pose2d truthStart,
      Pose2d estimatorStart,
      double linearVelocityMetersPerSecond,
      double angularVelocityRadPerSecond,
      int warmupCycles,
      int measuredCycles) {
    resetScenario(
        truthStart, estimatorStart, linearVelocityMetersPerSecond, angularVelocityRadPerSecond);
    for (int cycle = 0; cycle < warmupCycles; cycle++) {
      periodic();
    }
    resetMetrics();
    measuring = true;
    for (int cycle = 0; cycle < measuredCycles; cycle++) {
      periodic();
    }
    measuring = false;

    double meanError =
        acceptedConsumerCalls == 0 ? Double.NaN : captureErrorSumMeters / acceptedConsumerCalls;
    double maximumError = acceptedConsumerCalls == 0 ? Double.NaN : maxCaptureErrorMeters;
    return new ScenarioMetrics(
        name,
        detectionCount,
        acceptedConsumerCalls,
        selectedTimestamps.size(),
        meanError,
        maximumError,
        emittedStrategyCounts);
  }

  private void resetScenario(
      Pose2d truthStart,
      Pose2d estimatorStart,
      double linearVelocityMetersPerSecond,
      double angularVelocityRadPerSecond) {
    truthPose = new Pose2d(truthStart.getTranslation(), truthStart.getRotation());
    estimatorPose = new Pose2d(estimatorStart.getTranslation(), estimatorStart.getRotation());
    chassisSpeeds =
        new ChassisSpeeds(linearVelocityMetersPerSecond, 0.0, angularVelocityRadPerSecond);
    truthHistory.clear();
    estimatorHistory.clear();
    double nowSeconds = Timer.getFPGATimestamp();
    truthHistory.addSample(nowSeconds, truthPose);
    estimatorHistory.addSample(nowSeconds, estimatorPose);
  }

  private void resetMetrics() {
    detectionCount = 0;
    acceptedConsumerCalls = 0;
    captureErrorSumMeters = 0.0;
    maxCaptureErrorMeters = Double.NEGATIVE_INFINITY;
    selectedTimestamps.clear();
    emittedStrategyCounts.clear();
  }

  private void advanceState() {
    double deltaX = chassisSpeeds.vxMetersPerSecond * LOOP_PERIOD_SECONDS;
    double deltaHeading = chassisSpeeds.omegaRadiansPerSecond * LOOP_PERIOD_SECONDS;
    truthPose =
        new Pose2d(
            truthPose.getX() + deltaX,
            truthPose.getY(),
            truthPose.getRotation().plus(Rotation2d.fromRadians(deltaHeading)));
    estimatorPose =
        new Pose2d(
            estimatorPose.getX() + deltaX,
            estimatorPose.getY(),
            estimatorPose.getRotation().plus(Rotation2d.fromRadians(deltaHeading)));
    SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    double nowSeconds = Timer.getFPGATimestamp();
    truthHistory.addSample(nowSeconds, truthPose);
    estimatorHistory.addSample(nowSeconds, estimatorPose);
  }

  private void recordMeasurement(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations) {
    if (!measuring) {
      return;
    }
    acceptedConsumerCalls++;
    selectedTimestamps.add(timestampSeconds);
    Optional<Pose2d> captureTruth = truthHistory.getSample(timestampSeconds);
    if (captureTruth.isPresent()) {
      double errorMeters =
          pose.getTranslation().getDistance(captureTruth.orElseThrow().getTranslation());
      captureErrorSumMeters += errorMeters;
      maxCaptureErrorMeters = Math.max(maxCaptureErrorMeters, errorMeters);
    } else {
      captureErrorSumMeters = Double.NaN;
      maxCaptureErrorMeters = Double.NaN;
    }
  }

  public record ScenarioMetrics(
      String name,
      int detectionCount,
      int acceptedConsumerCalls,
      int distinctSelectedTimestamps,
      double meanCaptureTimeErrorMeters,
      double maxCaptureTimeErrorMeters,
      Map<PoseSolveStrategy, Integer> emittedStrategyCounts) {
    public ScenarioMetrics {
      emittedStrategyCounts = Map.copyOf(emittedStrategyCounts);
    }

    public int emittedStrategyCount(PoseSolveStrategy strategy) {
      return emittedStrategyCounts.getOrDefault(strategy, 0);
    }
  }

  public record HookOrderMetrics(long ownerUpdateCount, long firstCameraReadOwnerUpdateCount) {}

  private final class CountingVisionIO implements VisionIO {
    private final VisionIO delegate;

    private CountingVisionIO(VisionIO delegate) {
      this.delegate = delegate;
    }

    @Override
    public String getCameraName() {
      return delegate.getCameraName();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      if (firstCameraReadOwnerUpdateCount < 0) {
        firstCameraReadOwnerUpdateCount = owner.updateCount();
      }
      delegate.updateInputs(inputs);
      if (measuring) {
        VisionIO.PoseObservation[] observations = inputs.getPoseObservations();
        if (observations.length > 0) {
          detectionCount++;
        }
        for (VisionIO.PoseObservation observation : observations) {
          emittedStrategyCounts.merge(observation.strategy(), 1, Integer::sum);
        }
      }
    }

    @Override
    public void markVisionInitializationComplete() {
      delegate.markVisionInitializationComplete();
    }
  }

  private final class HeadingProvider implements VisionIOPhotonVision.HeadingProvider {
    @Override
    public Optional<Rotation2d> headingAt(double fpgaTimestampSeconds) {
      return estimatorHistory.getSample(fpgaTimestampSeconds).map(Pose2d::getRotation);
    }

    @Override
    public Optional<Pose3d> seedPoseAt(double fpgaTimestampSeconds) {
      return estimatorHistory.getSample(fpgaTimestampSeconds).map(Pose3d::new);
    }

    @Override
    public double angularRateRadPerSecond() {
      return chassisSpeeds.omegaRadiansPerSecond;
    }

    @Override
    public double linearSpeedMetersPerSecond() {
      return Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }
  }
}
