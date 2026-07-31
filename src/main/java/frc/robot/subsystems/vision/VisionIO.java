package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLog;

/** Replayable hardware boundary for one AprilTag vision camera. */
public interface VisionIO {
  record TargetObservation(Rotation2d tx, Rotation2d ty) {
    static final TargetObservation NONE = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
  }

  enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION,
    PHOTONVISION_MULTITAG_COPROCESSOR
  }

  enum PoseSolveStrategy {
    MULTI_TAG_PNP_ON_COPROCESSOR,
    CONSTRAINED_SOLVEPNP,
    PNP_DISTANCE_TRIG_SOLVE,
    LOWEST_AMBIGUITY,
    UNKNOWN
  }

  record PoseObservation(
      double timestampSeconds,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double confidenceDistanceMeters,
      PoseObservationType type,
      PoseSolveStrategy strategy,
      int[] tagIds) {
    public PoseObservation {
      tagIds = Arrays.copyOf(tagIds, tagIds.length);
    }

    @Override
    public int[] tagIds() {
      return Arrays.copyOf(tagIds, tagIds.length);
    }
  }

  /** AutoLog-compatible pose data stored separately from the variable-length tag-ID sidecar. */
  record PoseObservationLog(
      double timestampSeconds,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double confidenceDistanceMeters,
      PoseObservationType type,
      PoseSolveStrategy strategy) {}

  @AutoLog
  class VisionIOInputs {
    public String cameraName = "";
    public boolean connected = false;
    public TargetObservation latestTargetObservation = TargetObservation.NONE;
    public PoseObservationLog[] poseObservationLogs = new PoseObservationLog[0];
    public int[][] poseObservationTagIds = new int[0][];

    /** Stores observations in the AutoLog-compatible record-plus-sidecar representation. */
    public void setPoseObservations(PoseObservation[] poseObservations) {
      if (poseObservations == null) {
        poseObservationLogs = new PoseObservationLog[0];
        poseObservationTagIds = new int[0][];
        return;
      }

      poseObservationLogs = new PoseObservationLog[poseObservations.length];
      poseObservationTagIds = new int[poseObservations.length][];
      for (int index = 0; index < poseObservations.length; index++) {
        PoseObservation observation = poseObservations[index];
        poseObservationLogs[index] =
            new PoseObservationLog(
                observation.timestampSeconds(),
                observation.pose(),
                observation.ambiguity(),
                observation.tagCount(),
                observation.confidenceDistanceMeters(),
                observation.type(),
                observation.strategy());
        poseObservationTagIds[index] = observation.tagIds();
      }
    }

    /** Reconstructs immutable runtime observations from the AutoLog storage representation. */
    public PoseObservation[] getPoseObservations() {
      if (poseObservationLogs == null) {
        return new PoseObservation[0];
      }

      PoseObservation[] observations = new PoseObservation[poseObservationLogs.length];
      for (int index = 0; index < poseObservationLogs.length; index++) {
        PoseObservationLog observationLog = poseObservationLogs[index];
        int[] tagIds =
            poseObservationTagIds != null
                    && index < poseObservationTagIds.length
                    && poseObservationTagIds[index] != null
                ? poseObservationTagIds[index]
                : new int[0];
        observations[index] =
            new PoseObservation(
                observationLog.timestampSeconds(),
                observationLog.pose(),
                observationLog.ambiguity(),
                observationLog.tagCount(),
                observationLog.confidenceDistanceMeters(),
                observationLog.type(),
                observationLog.strategy(),
                tagIds);
      }
      return observations;
    }
  }

  class NoOp implements VisionIO {
    private final String cameraName;

    public NoOp(String cameraName) {
      this.cameraName = cameraName;
    }

    @Override
    public String getCameraName() {
      return cameraName;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.cameraName = cameraName;
      inputs.connected = false;
      inputs.latestTargetObservation = TargetObservation.NONE;
      inputs.setPoseObservations(new PoseObservation[0]);
    }
  }

  String getCameraName();

  default void updateInputs(VisionIOInputs inputs) {}

  default void markVisionInitializationComplete() {}
}
