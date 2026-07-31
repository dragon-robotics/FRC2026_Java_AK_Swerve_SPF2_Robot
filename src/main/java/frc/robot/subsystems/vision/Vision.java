package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConsensus.Candidate;
import frc.robot.subsystems.vision.VisionConsensus.RejectedCandidate;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.util.constants.FieldConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Replay-safe orchestration, filtering, and consensus for AprilTag observations. */
public class Vision extends SubsystemBase {
  private static final Pose3d[] NO_POSES = new Pose3d[0];
  private static final int[] NO_TAG_IDS = new int[0];

  private final VisionDriveBindings drive;
  private final VisionRuntimeConfig config;
  private final BooleanSupplier disabled;
  private final DoubleSupplier nowSeconds;
  private final Runnable beforeCameraInputs;

  @SuppressWarnings("unused")
  private final boolean releaseStartupStrategyOnInitialization;

  private final VisionIO[] io;
  private final String[] cameraNames;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final CameraLoopLog[] cameraLogs;

  private final List<Pose3d> rawRobotPoses = new ArrayList<>();
  private final List<Pose3d> acceptedRobotPoses = new ArrayList<>();
  private final List<Pose3d> rejectedRobotPoses = new ArrayList<>();
  private final List<Pose3d> selectedRobotPoses = new ArrayList<>();
  private final List<Candidate> candidates = new ArrayList<>();
  private final List<Pose3d> acceptedTagToPoseLines = new ArrayList<>();
  private List<Candidate> coherentCandidates = List.of();
  private Optional<Candidate> selectedConsensusCandidate = Optional.empty();
  private boolean aiming;

  public Vision(
      VisionDriveBindings drive,
      VisionRuntimeConfig config,
      Runnable beforeCameraInputs,
      VisionIO... io) {
    this(
        drive,
        config,
        DriverStation::isDisabled,
        Timer::getFPGATimestamp,
        beforeCameraInputs,
        true,
        io);
  }

  Vision(
      VisionDriveBindings drive,
      VisionRuntimeConfig config,
      BooleanSupplier disabled,
      DoubleSupplier nowSeconds,
      Runnable beforeCameraInputs,
      boolean releaseStartupStrategyOnInitialization,
      VisionIO... io) {
    this.drive = drive;
    this.config = config;
    this.disabled = disabled;
    this.nowSeconds = nowSeconds;
    this.beforeCameraInputs = beforeCameraInputs;
    this.releaseStartupStrategyOnInitialization = releaseStartupStrategyOnInitialization;
    this.io = Arrays.copyOf(io, io.length);
    cameraNames = new String[io.length];
    inputs = new VisionIOInputsAutoLogged[io.length];
    disconnectedAlerts = new Alert[io.length];
    cameraLogs = new CameraLoopLog[io.length];
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      cameraNames[cameraIndex] = io[cameraIndex].getCameraName();
      inputs[cameraIndex] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[cameraIndex] =
          new Alert(
              "Vision camera " + cameraNames[cameraIndex] + " is disconnected.",
              AlertType.kWarning);
      cameraLogs[cameraIndex] = new CameraLoopLog();
    }
  }

  /** Tightens translation uncertainty while the robot is actively aiming. */
  public void setAiming(boolean aiming) {
    this.aiming = aiming;
  }

  @Override
  public void periodic() {
    long loopStartNanoseconds = System.nanoTime();
    double now = nowSeconds.getAsDouble();
    beforeCameraInputs.run();
    clearLoopState();

    boolean isDisabled = disabled.getAsBoolean();
    ChassisSpeeds speeds = drive.chassisSpeeds().get();
    double pitchDegrees = drive.pitchDegrees().getAsDouble();
    double rollDegrees = drive.rollDegrees().getAsDouble();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      updateCameraInputs(cameraIndex, pitchDegrees, rollDegrees);
    }
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      processCameraObservations(cameraIndex, now, isDisabled, pitchDegrees, rollDegrees);
    }
    selectAndConsume(now);
    recordOutputs(speeds, (System.nanoTime() - loopStartNanoseconds) / 1_000_000.0);
  }

  private void clearLoopState() {
    rawRobotPoses.clear();
    acceptedRobotPoses.clear();
    rejectedRobotPoses.clear();
    selectedRobotPoses.clear();
    candidates.clear();
    acceptedTagToPoseLines.clear();
    coherentCandidates = List.of();
    selectedConsensusCandidate = Optional.empty();
    for (CameraLoopLog cameraLog : cameraLogs) {
      cameraLog.clear();
    }
  }

  private void updateCameraInputs(int cameraIndex, double pitchDegrees, double rollDegrees) {
    io[cameraIndex].updateInputs(inputs[cameraIndex]);
    Logger.processInputs("Vision/" + cameraNames[cameraIndex], inputs[cameraIndex]);
    disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

    CameraLoopLog cameraLog = cameraLogs[cameraIndex];
    cameraLog.pitchDegrees = pitchDegrees;
    cameraLog.rollDegrees = rollDegrees;
    cameraLog.tagIds = inputs[cameraIndex].getTagIds();
    for (int tagId : cameraLog.tagIds) {
      FieldConstants.APTAG_FIELD_LAYOUT.getTagPose(tagId).ifPresent(cameraLog.tagPoses::add);
    }
  }

  private void processCameraObservations(
      int cameraIndex, double now, boolean isDisabled, double pitchDegrees, double rollDegrees) {
    CameraLoopLog cameraLog = cameraLogs[cameraIndex];
    for (PoseObservation observation : inputs[cameraIndex].getPoseObservations()) {
      cameraLog.rawRobotPoses.add(observation.pose());
      rawRobotPoses.add(observation.pose());
      cameraLog.activeStrategy = observation.strategy().name();
      cameraLog.observationType = observation.type().name();
      cameraLog.confidenceDistanceMeters = observation.confidenceDistanceMeters();
      cameraLog.ambiguity = observation.ambiguity();
      cameraLog.captureTimestampSeconds = observation.timestampSeconds();
      cameraLog.timestampSkewSeconds = now - observation.timestampSeconds();

      Optional<String> timestampRejection =
          VisionConsensus.timestampRejectionReason(observation.timestampSeconds(), now);
      if (timestampRejection.isPresent()) {
        cameraLog.skewRejectedRobotPoses.add(observation.pose());
        reject(cameraLog, observation.pose(), timestampRejection.orElseThrow(), false);
        cameraLog.rejectionReason = timestampRejection.orElseThrow();
        continue;
      }

      Pose2d referencePose =
          drive
              .timestampedPose()
              .apply(observation.timestampSeconds())
              .orElseGet(drive.currentPose());
      Pose2d visionPose = observation.pose().toPose2d();
      double innovationMeters =
          visionPose.getTranslation().getDistance(referencePose.getTranslation());
      cameraLog.innovationMeters = innovationMeters;
      if (isDisabled && innovationMeters > VisionConstants.MAX_POSE_INNOVATION_METERS) {
        cameraLog.innovationBypassedInDisabledMeters = innovationMeters;
      }

      Optional<String> hardRejection =
          VisionFilter.rejectionReason(
              observation,
              FieldConstants.APTAG_FIELD_LAYOUT,
              pitchDegrees,
              rollDegrees,
              isDisabled,
              referencePose,
              config);
      if (hardRejection.isPresent()) {
        reject(cameraLog, observation.pose(), hardRejection.orElseThrow(), true);
        cameraLog.rejectionReason = hardRejection.orElseThrow();
        continue;
      }

      Candidate candidate =
          new Candidate(
              cameraIndex,
              cameraNames[cameraIndex],
              observation,
              visionPose,
              VisionFilter.standardDeviations(
                  observation,
                  VisionConstants.CAMERAS.get(
                      Math.min(cameraIndex, VisionConstants.CAMERAS.size() - 1)),
                  aiming,
                  FieldConstants.APTAG_FIELD_LAYOUT,
                  config),
              innovationMeters);
      candidates.add(candidate);
      cameraLog.acceptedRobotPoses.add(observation.pose());
      acceptedRobotPoses.add(observation.pose());
    }
  }

  private void selectAndConsume(double now) {
    VisionConsensus.TemporalSelection temporalSelection =
        VisionConsensus.selectTimestampCoherent(candidates, now);
    coherentCandidates = temporalSelection.selectedCandidates();
    for (RejectedCandidate rejected : temporalSelection.rejectedCandidates()) {
      CameraLoopLog cameraLog = cameraLogs[rejected.candidate().cameraIndex()];
      if (VisionConsensus.SUPERSEDED_BY_NEWER_CAMERA_FRAME.equals(rejected.reason())) {
        cameraLog.supersededRobotPoses.add(rejected.candidate().observation().pose());
        reject(cameraLog, rejected.candidate().observation().pose(), rejected.reason(), false);
      } else if (VisionConsensus.TIMESTAMP_CLUSTER_NOT_SELECTED.equals(rejected.reason())) {
        cameraLog.skewRejectedRobotPoses.add(rejected.candidate().observation().pose());
        reject(cameraLog, rejected.candidate().observation().pose(), rejected.reason(), false);
      } else if (VisionConsensus.INVALID_TIMESTAMP.equals(rejected.reason())
          || VisionConsensus.FUTURE_TIMESTAMP.equals(rejected.reason())
          || VisionConsensus.STALE_TIMESTAMP.equals(rejected.reason())) {
        cameraLog.skewRejectedRobotPoses.add(rejected.candidate().observation().pose());
        reject(cameraLog, rejected.candidate().observation().pose(), rejected.reason(), false);
      } else {
        reject(cameraLog, rejected.candidate().observation().pose(), rejected.reason(), true);
      }
    }

    Optional<Candidate> selected =
        VisionConsensus.selectSpatialConsensus(temporalSelection.selectedCandidates());
    selectedConsensusCandidate = selected;
    if (selected.isEmpty()) {
      return;
    }

    Candidate winner = selected.orElseThrow();
    for (Candidate candidate : temporalSelection.selectedCandidates()) {
      if (candidate != winner) {
        reject(
            cameraLogs[candidate.cameraIndex()],
            candidate.observation().pose(),
            "CONSENSUS_NOT_SELECTED",
            true);
      }
    }

    CameraLoopLog winnerLog = cameraLogs[winner.cameraIndex()];
    winnerLog.selectedRobotPoses.add(winner.observation().pose());
    selectedRobotPoses.add(winner.observation().pose());
    for (int tagId : winner.observation().tagIds()) {
      FieldConstants.APTAG_FIELD_LAYOUT
          .getTagPose(tagId)
          .ifPresent(
              tagPose -> {
                acceptedTagToPoseLines.add(tagPose);
                acceptedTagToPoseLines.add(winner.observation().pose());
              });
    }
    drive
        .measurementConsumer()
        .accept(
            winner.visionPose(),
            winner.observation().timestampSeconds(),
            winner.standardDeviations());
  }

  private void reject(
      CameraLoopLog cameraLog, Pose3d pose, String reason, boolean regularRejection) {
    if (regularRejection) {
      cameraLog.rejectedRobotPoses.add(pose);
    }
    rejectedRobotPoses.add(pose);
    if (cameraLog.rejectionReason.isEmpty()) {
      cameraLog.rejectionReason = reason;
    }
  }

  private void recordOutputs(ChassisSpeeds speeds, double loopExecutionMilliseconds) {
    for (int cameraIndex = 0; cameraIndex < cameraNames.length; cameraIndex++) {
      String key = "Vision/" + cameraNames[cameraIndex] + "/";
      CameraLoopLog cameraLog = cameraLogs[cameraIndex];
      Logger.recordOutput(key + "RawRobotPoses", poses(cameraLog.rawRobotPoses));
      Logger.recordOutput(key + "AcceptedRobotPoses", poses(cameraLog.acceptedRobotPoses));
      Logger.recordOutput(key + "RejectedRobotPoses", poses(cameraLog.rejectedRobotPoses));
      Logger.recordOutput(key + "SupersededRobotPoses", poses(cameraLog.supersededRobotPoses));
      Logger.recordOutput(key + "SkewRejectedRobotPoses", poses(cameraLog.skewRejectedRobotPoses));
      Logger.recordOutput(key + "SelectedRobotPoses", poses(cameraLog.selectedRobotPoses));
      Logger.recordOutput(key + "TagPoses", poses(cameraLog.tagPoses));
      Logger.recordOutput(key + "TagIDs", cameraLog.tagIds);
      Logger.recordOutput(key + "ActiveStrategy", cameraLog.activeStrategy);
      Logger.recordOutput(key + "ObservationType", cameraLog.observationType);
      Logger.recordOutput(key + "RejectionReason", cameraLog.rejectionReason);
      Logger.recordOutput(key + "ConfidenceDistanceMeters", cameraLog.confidenceDistanceMeters);
      Logger.recordOutput(key + "Ambiguity", cameraLog.ambiguity);
      Logger.recordOutput(key + "CaptureTimestampSeconds", cameraLog.captureTimestampSeconds);
      Logger.recordOutput(key + "TimestampSkewSeconds", cameraLog.timestampSkewSeconds);
      Logger.recordOutput(key + "PitchDegrees", cameraLog.pitchDegrees);
      Logger.recordOutput(key + "RollDegrees", cameraLog.rollDegrees);
      Logger.recordOutput(key + "InnovationMeters", cameraLog.innovationMeters);
      Logger.recordOutput(
          key + "InnovationBypassedInDisabledMeters", cameraLog.innovationBypassedInDisabledMeters);
    }

    Optional<Candidate> selected = selectedConsensusCandidate;
    Logger.recordOutput("Vision/RawRobotPoses", poses(rawRobotPoses));
    Logger.recordOutput("Vision/AcceptedRobotPoses", poses(acceptedRobotPoses));
    Logger.recordOutput("Vision/RejectedRobotPoses", poses(rejectedRobotPoses));
    Logger.recordOutput("Vision/SelectedRobotPoses", poses(selectedRobotPoses));
    Logger.recordOutput(
        "Vision/Consensus/SelectedCamera", selected.map(Candidate::cameraName).orElse(""));
    Logger.recordOutput("Vision/Consensus/CandidateCount", candidates.size());
    Logger.recordOutput(
        "Vision/Consensus/SelectedClusterSize",
        selected.map(candidate -> clusterSize(candidate, coherentCandidates)).orElse(0));
    Logger.recordOutput(
        "Vision/Consensus/SelectedStdDevMeters",
        selected.map(Candidate::linearStdDevMeters).orElse(0.0));
    Logger.recordOutput(
        "Vision/Consensus/SelectedInnovationMeters",
        selected.map(Candidate::innovationMeters).orElse(0.0));
    Logger.recordOutput(
        "Vision/Consensus/SelectedTimestampSeconds",
        selected.map(candidate -> candidate.observation().timestampSeconds()).orElse(0.0));
    Logger.recordOutput("Vision/Aiming", aiming);
    Logger.recordOutput(
        "Vision/LinearSpeedMetersPerSecond",
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    Logger.recordOutput("Vision/AngularSpeedRadiansPerSecond", speeds.omegaRadiansPerSecond);
    Logger.recordOutput("Vision/AcceptedFieldPoses", poses(selectedRobotPoses));
    Logger.recordOutput("Vision/AcceptedTagToPoseLines", poses(acceptedTagToPoseLines));
    Logger.recordOutput("Vision/LoopExecutionMilliseconds", loopExecutionMilliseconds);
    Logger.recordOutput("Vision/Config/StrategyMode", config.strategyMode());
    Logger.recordOutput("Vision/Config/StartupStrategyOrder", config.startupStrategyOrder().name());
    Logger.recordOutput(
        "Vision/Config/ExplicitStrategyOrder",
        config.configuredStrategyOrder().stream()
            .map(Enum::name)
            .reduce((a, b) -> a + "," + b)
            .orElse(""));
    Logger.recordOutput(
        "Vision/Config/TagDistanceConfidenceMode", config.tagDistanceConfidenceMode().name());
  }

  private static int clusterSize(Candidate candidate, List<Candidate> coherentCandidates) {
    int count = 0;
    for (Candidate other : coherentCandidates) {
      if (candidate.visionPose().getTranslation().getDistance(other.visionPose().getTranslation())
          <= VisionConstants.CONSENSUS_RADIUS_METERS) {
        count++;
      }
    }
    return count;
  }

  private static Pose3d[] poses(List<Pose3d> poses) {
    return poses.isEmpty() ? NO_POSES : poses.toArray(Pose3d[]::new);
  }

  private static final class CameraLoopLog {
    private final List<Pose3d> rawRobotPoses = new ArrayList<>();
    private final List<Pose3d> acceptedRobotPoses = new ArrayList<>();
    private final List<Pose3d> rejectedRobotPoses = new ArrayList<>();
    private final List<Pose3d> supersededRobotPoses = new ArrayList<>();
    private final List<Pose3d> skewRejectedRobotPoses = new ArrayList<>();
    private final List<Pose3d> selectedRobotPoses = new ArrayList<>();
    private final List<Pose3d> tagPoses = new ArrayList<>();
    private int[] tagIds = NO_TAG_IDS;
    private String activeStrategy = "";
    private String observationType = "";
    private String rejectionReason = "";
    private double confidenceDistanceMeters;
    private double ambiguity;
    private double captureTimestampSeconds;
    private double timestampSkewSeconds;
    private double pitchDegrees;
    private double rollDegrees;
    private double innovationMeters;
    private double innovationBypassedInDisabledMeters;

    private void clear() {
      rawRobotPoses.clear();
      acceptedRobotPoses.clear();
      rejectedRobotPoses.clear();
      supersededRobotPoses.clear();
      skewRejectedRobotPoses.clear();
      selectedRobotPoses.clear();
      tagPoses.clear();
      tagIds = NO_TAG_IDS;
      activeStrategy = "";
      observationType = "";
      rejectionReason = "";
      confidenceDistanceMeters = 0.0;
      ambiguity = 0.0;
      captureTimestampSeconds = 0.0;
      timestampSkewSeconds = 0.0;
      pitchDegrees = 0.0;
      rollDegrees = 0.0;
      innovationMeters = 0.0;
      innovationBypassedInDisabledMeters = 0.0;
    }
  }
}
