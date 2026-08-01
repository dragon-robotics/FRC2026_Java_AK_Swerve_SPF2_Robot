package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseSolveStrategy;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.vision.VisionRuntimeConfig.StartupStrategyOrder;
import frc.robot.subsystems.vision.VisionStartupStrategyComparisonSupport.StrategyMetrics;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.List;
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
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Drive.getModuleTranslations());
  private final VisionSimulation owner;
  private final CountingVisionIO[] cameraIO;
  private final Vision vision;
  private final HeadingProvider headingProvider = new HeadingProvider();
  private final StartupStrategyOrder startupStrategyOrder;
  private final Set<Double> selectedTimestamps = new HashSet<>();
  private final EnumMap<PoseSolveStrategy, Integer> emittedStrategyCounts =
      new EnumMap<>(PoseSolveStrategy.class);

  private Pose2d truthPose;
  private SwerveModulePosition[] modulePositions = zeroModulePositions();
  private SwerveDrivePoseEstimator poseEstimator;
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private long firstCameraReadOwnerUpdateCount = -1;
  private boolean measuring;
  private int detectionCount;
  private int acceptedConsumerCalls;
  private double captureErrorSumMeters;
  private double maxCaptureErrorMeters = Double.NEGATIVE_INFINITY;
  private boolean movingScenario;
  private double maximumEstimatorStepMeters;
  private double maximumMovingExcessJumpMeters;
  private int movingExcessJumpOverFortyCentimetersCount;

  public VisionSimulationHarness(
      String instanceName, Pose2d initialTruth, VisionRuntimeConfig runtimeConfig) {
    truthPose = initialTruth;
    startupStrategyOrder = runtimeConfig.startupStrategyOrder();
    poseEstimator = createPoseEstimator(initialTruth);
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
            this::resetEstimator,
            () -> poseEstimator.getEstimatedPosition(),
            timestampSeconds -> poseEstimator.sampleAt(timestampSeconds),
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
    Pose2d previousTruthPose = truthPose;
    Pose2d previousEstimatorPose = poseEstimator.getEstimatedPosition();
    advanceState();
    vision.periodic();
    if (measuring) {
      recordEstimatorStep(
          previousTruthPose,
          truthPose,
          previousEstimatorPose,
          poseEstimator.getEstimatedPosition());
    }
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

  /** Runs the approved eight stationary and two moving startup-order comparison scenarios. */
  public StrategyMetrics runStartupStrategyComparison(
      List<Pose2d> stationaryPoses, int warmupCycles, int measuredCycles) {
    if (stationaryPoses.size() != 8) {
      throw new IllegalArgumentException(
          "Startup comparison requires exactly eight stationary poses");
    }

    double maximumStationaryJump = 0.0;
    double maximumMovingExcessJump = 0.0;
    int movingExcessJumpOverFortyCentimeters = 0;
    int totalAcceptedObservations = 0;
    double totalAcceptedErrorMeters = 0.0;
    double worstAcceptedErrorMeters = Double.NEGATIVE_INFINITY;

    for (int index = 0; index < stationaryPoses.size(); index++) {
      ScenarioMetrics metrics =
          runStationary(
              "startup-stationary-" + index,
              stationaryPoses.get(index),
              warmupCycles,
              measuredCycles);
      maximumStationaryJump = Math.max(maximumStationaryJump, metrics.maximumEstimatorStepMeters());
      totalAcceptedObservations += metrics.acceptedConsumerCalls();
      totalAcceptedErrorMeters += metrics.captureErrorSumMeters();
      worstAcceptedErrorMeters =
          Math.max(worstAcceptedErrorMeters, metrics.maxCaptureTimeErrorMeters());
    }

    ScenarioMetrics translation =
        runMoving(
            "startup-translation",
            stationaryPoses.get(0),
            0.65,
            0.25,
            warmupCycles,
            measuredCycles);
    ScenarioMetrics rotation =
        runMoving(
            "startup-rotation", stationaryPoses.get(4), 0.0, 0.40, warmupCycles, measuredCycles);
    for (ScenarioMetrics metrics : List.of(translation, rotation)) {
      maximumMovingExcessJump =
          Math.max(maximumMovingExcessJump, metrics.maximumMovingExcessJumpMeters());
      movingExcessJumpOverFortyCentimeters += metrics.movingExcessJumpOverFortyCentimetersCount();
      totalAcceptedObservations += metrics.acceptedConsumerCalls();
      totalAcceptedErrorMeters += metrics.captureErrorSumMeters();
      worstAcceptedErrorMeters =
          Math.max(worstAcceptedErrorMeters, metrics.maxCaptureTimeErrorMeters());
    }

    int measuredSchedulerCycles = (stationaryPoses.size() + 2) * measuredCycles;
    return new StrategyMetrics(
        startupStrategyOrder,
        maximumStationaryJump,
        maximumMovingExcessJump,
        movingExcessJumpOverFortyCentimeters,
        totalAcceptedObservations / (double) measuredSchedulerCycles,
        totalAcceptedObservations,
        totalAcceptedObservations == 0
            ? Double.NaN
            : totalAcceptedErrorMeters / totalAcceptedObservations,
        totalAcceptedObservations == 0 ? Double.NaN : worstAcceptedErrorMeters);
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
        measuredCycles,
        captureErrorSumMeters,
        meanError,
        maximumError,
        maximumEstimatorStepMeters,
        maximumMovingExcessJumpMeters,
        movingExcessJumpOverFortyCentimetersCount,
        emittedStrategyCounts);
  }

  private void resetScenario(
      Pose2d truthStart,
      Pose2d estimatorStart,
      double linearVelocityMetersPerSecond,
      double angularVelocityRadPerSecond) {
    truthPose = new Pose2d(truthStart.getTranslation(), truthStart.getRotation());
    modulePositions = zeroModulePositions();
    poseEstimator = createPoseEstimator(estimatorStart);
    chassisSpeeds =
        new ChassisSpeeds(linearVelocityMetersPerSecond, 0.0, angularVelocityRadPerSecond);
    movingScenario = linearVelocityMetersPerSecond != 0.0 || angularVelocityRadPerSecond != 0.0;
    truthHistory.clear();
    double nowSeconds = Timer.getFPGATimestamp();
    truthHistory.addSample(nowSeconds, truthPose);
  }

  private void resetMetrics() {
    detectionCount = 0;
    acceptedConsumerCalls = 0;
    captureErrorSumMeters = 0.0;
    maxCaptureErrorMeters = Double.NEGATIVE_INFINITY;
    maximumEstimatorStepMeters = 0.0;
    maximumMovingExcessJumpMeters = 0.0;
    movingExcessJumpOverFortyCentimetersCount = 0;
    selectedTimestamps.clear();
    emittedStrategyCounts.clear();
  }

  private void advanceState() {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    for (int index = 0; index < modulePositions.length; index++) {
      modulePositions[index] =
          new SwerveModulePosition(
              modulePositions[index].distanceMeters
                  + moduleStates[index].speedMetersPerSecond * LOOP_PERIOD_SECONDS,
              moduleStates[index].angle);
    }
    truthPose =
        truthPose.exp(
            new Twist2d(
                chassisSpeeds.vxMetersPerSecond * LOOP_PERIOD_SECONDS,
                chassisSpeeds.vyMetersPerSecond * LOOP_PERIOD_SECONDS,
                chassisSpeeds.omegaRadiansPerSecond * LOOP_PERIOD_SECONDS));
    SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    double nowSeconds = Timer.getFPGATimestamp();
    truthHistory.addSample(nowSeconds, truthPose);
    poseEstimator.updateWithTime(nowSeconds, truthPose.getRotation(), modulePositions);
  }

  private void recordMeasurement(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations) {
    poseEstimator.addVisionMeasurement(pose, timestampSeconds, standardDeviations);
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

  private void recordEstimatorStep(
      Pose2d previousTruth,
      Pose2d currentTruth,
      Pose2d previousEstimator,
      Pose2d currentEstimator) {
    double estimatorStepMeters =
        previousEstimator.getTranslation().getDistance(currentEstimator.getTranslation());
    maximumEstimatorStepMeters = Math.max(maximumEstimatorStepMeters, estimatorStepMeters);
    if (!movingScenario) {
      return;
    }

    double truthStepMeters =
        previousTruth.getTranslation().getDistance(currentTruth.getTranslation());
    double excessJumpMeters = Math.max(0.0, estimatorStepMeters - truthStepMeters);
    maximumMovingExcessJumpMeters = Math.max(maximumMovingExcessJumpMeters, excessJumpMeters);
    if (excessJumpMeters > 0.40) {
      movingExcessJumpOverFortyCentimetersCount++;
    }
  }

  private SwerveDrivePoseEstimator createPoseEstimator(Pose2d initialPose) {
    return new SwerveDrivePoseEstimator(
        kinematics, truthPose.getRotation(), modulePositions, initialPose);
  }

  private void resetEstimator(Pose2d pose) {
    poseEstimator.resetPosition(truthPose.getRotation(), modulePositions, pose);
  }

  private static SwerveModulePosition[] zeroModulePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
  }

  public record ScenarioMetrics(
      String name,
      int detectionCount,
      int acceptedConsumerCalls,
      int distinctSelectedTimestamps,
      int measuredCycleCount,
      double captureErrorSumMeters,
      double meanCaptureTimeErrorMeters,
      double maxCaptureTimeErrorMeters,
      double maximumEstimatorStepMeters,
      double maximumMovingExcessJumpMeters,
      int movingExcessJumpOverFortyCentimetersCount,
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
      return poseEstimator.sampleAt(fpgaTimestampSeconds).map(Pose2d::getRotation);
    }

    @Override
    public Optional<Pose3d> seedPoseAt(double fpgaTimestampSeconds) {
      return poseEstimator.sampleAt(fpgaTimestampSeconds).map(Pose3d::new);
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
