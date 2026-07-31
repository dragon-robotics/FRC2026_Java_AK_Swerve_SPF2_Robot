package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

/** Timestamp and pose-consensus selection for independently decoded camera observations. */
public final class VisionConsensus {
  public static final String INVALID_TIMESTAMP = "INVALID_TIMESTAMP";
  public static final String FUTURE_TIMESTAMP = "FUTURE_TIMESTAMP";
  public static final String STALE_TIMESTAMP = "STALE_TIMESTAMP";
  public static final String SUPERSEDED_BY_NEWER_CAMERA_FRAME = "SUPERSEDED_BY_NEWER_CAMERA_FRAME";
  public static final String TIMESTAMP_CLUSTER_NOT_SELECTED = "TIMESTAMP_CLUSTER_NOT_SELECTED";

  public record Candidate(
      int cameraIndex,
      String cameraName,
      PoseObservation observation,
      Pose2d visionPose,
      Matrix<N3, N1> standardDeviations,
      double innovationMeters) {
    public double linearStdDevMeters() {
      return standardDeviations.get(0, 0);
    }
  }

  public record RejectedCandidate(Candidate candidate, String reason) {}

  public record TemporalSelection(
      List<Candidate> selectedCandidates, List<RejectedCandidate> rejectedCandidates) {
    public TemporalSelection {
      selectedCandidates = List.copyOf(selectedCandidates);
      rejectedCandidates = List.copyOf(rejectedCandidates);
    }
  }

  private VisionConsensus() {}

  public static Optional<String> timestampRejectionReason(
      double timestampSeconds, double nowSeconds) {
    if (!Double.isFinite(timestampSeconds)) {
      return Optional.of(INVALID_TIMESTAMP);
    }
    if (timestampSeconds > nowSeconds + VisionConstants.MAX_FUTURE_TIMESTAMP_SECONDS) {
      return Optional.of(FUTURE_TIMESTAMP);
    }
    if (timestampSeconds < nowSeconds - VisionConstants.MAX_OBSERVATION_AGE_SECONDS) {
      return Optional.of(STALE_TIMESTAMP);
    }
    return Optional.empty();
  }

  public static TemporalSelection selectTimestampCoherent(
      List<Candidate> validCandidates, double nowSeconds) {
    List<Candidate> newestPerCamera = new ArrayList<>();
    List<RejectedCandidate> rejectedCandidates = new ArrayList<>();

    for (Candidate candidate : validCandidates) {
      Optional<String> timestampRejection =
          timestampRejectionReason(candidate.observation().timestampSeconds(), nowSeconds);
      if (timestampRejection.isPresent()) {
        rejectedCandidates.add(new RejectedCandidate(candidate, timestampRejection.orElseThrow()));
        continue;
      }

      int retainedIndex = retainedCameraIndex(newestPerCamera, candidate.cameraIndex());
      if (retainedIndex < 0) {
        newestPerCamera.add(candidate);
        continue;
      }

      Candidate retained = newestPerCamera.get(retainedIndex);
      if (candidate.observation().timestampSeconds() > retained.observation().timestampSeconds()) {
        rejectedCandidates.add(new RejectedCandidate(retained, SUPERSEDED_BY_NEWER_CAMERA_FRAME));
        newestPerCamera.set(retainedIndex, candidate);
      } else {
        rejectedCandidates.add(new RejectedCandidate(candidate, SUPERSEDED_BY_NEWER_CAMERA_FRAME));
      }
    }

    if (newestPerCamera.isEmpty()) {
      return new TemporalSelection(List.of(), rejectedCandidates);
    }

    List<Candidate> winningWindow = null;
    for (Candidate windowStart : newestPerCamera) {
      double startTimestamp = windowStart.observation().timestampSeconds();
      List<Candidate> window = new ArrayList<>();
      for (Candidate candidate : newestPerCamera) {
        double timestamp = candidate.observation().timestampSeconds();
        if (timestamp >= startTimestamp
            && timestamp <= startTimestamp + VisionConstants.MAX_CONSENSUS_TIMESTAMP_SPAN_SECONDS) {
          window.add(candidate);
        }
      }
      if (winningWindow == null || compareTemporalWindows(window, winningWindow) > 0) {
        winningWindow = window;
      }
    }

    for (Candidate candidate : newestPerCamera) {
      if (!winningWindow.contains(candidate)) {
        rejectedCandidates.add(new RejectedCandidate(candidate, TIMESTAMP_CLUSTER_NOT_SELECTED));
      }
    }
    return new TemporalSelection(winningWindow, rejectedCandidates);
  }

  public static Optional<Candidate> selectSpatialConsensus(List<Candidate> coherentCandidates) {
    Candidate winner = null;
    int winningNeighborhoodSize = 0;
    double winningQuality = Double.POSITIVE_INFINITY;

    for (Candidate candidate : coherentCandidates) {
      int neighborhoodSize = 0;
      double quality = candidate.linearStdDevMeters();
      for (Candidate neighbor : coherentCandidates) {
        double distanceMeters =
            candidate
                .visionPose()
                .getTranslation()
                .getDistance(neighbor.visionPose().getTranslation());
        if (distanceMeters <= VisionConstants.CONSENSUS_RADIUS_METERS) {
          neighborhoodSize++;
          quality += distanceMeters + neighbor.linearStdDevMeters();
        }
      }

      if (winner == null
          || neighborhoodSize > winningNeighborhoodSize
          || (neighborhoodSize == winningNeighborhoodSize && quality < winningQuality)
          || (neighborhoodSize == winningNeighborhoodSize
              && Double.compare(quality, winningQuality) == 0
              && candidate.cameraIndex() < winner.cameraIndex())) {
        winner = candidate;
        winningNeighborhoodSize = neighborhoodSize;
        winningQuality = quality;
      }
    }
    return Optional.ofNullable(winner);
  }

  private static int retainedCameraIndex(List<Candidate> candidates, int cameraIndex) {
    for (int index = 0; index < candidates.size(); index++) {
      if (candidates.get(index).cameraIndex() == cameraIndex) {
        return index;
      }
    }
    return -1;
  }

  private static int compareTemporalWindows(List<Candidate> first, List<Candidate> second) {
    int cameraCount = Integer.compare(first.size(), second.size());
    if (cameraCount != 0) {
      return cameraCount;
    }

    double firstMaximumTimestamp = maximumTimestamp(first);
    double secondMaximumTimestamp = maximumTimestamp(second);
    int maximumTimestamp = Double.compare(firstMaximumTimestamp, secondMaximumTimestamp);
    if (maximumTimestamp != 0) {
      return maximumTimestamp;
    }

    int standardDeviationSum =
        Double.compare(sumLinearStandardDeviations(second), sumLinearStandardDeviations(first));
    if (standardDeviationSum != 0) {
      return standardDeviationSum;
    }

    List<Integer> firstCameraIndices = cameraIndices(first);
    List<Integer> secondCameraIndices = cameraIndices(second);
    for (int index = 0; index < firstCameraIndices.size(); index++) {
      int cameraIndex =
          Integer.compare(secondCameraIndices.get(index), firstCameraIndices.get(index));
      if (cameraIndex != 0) {
        return cameraIndex;
      }
    }
    return 0;
  }

  private static double maximumTimestamp(List<Candidate> candidates) {
    double maximum = Double.NEGATIVE_INFINITY;
    for (Candidate candidate : candidates) {
      maximum = Math.max(maximum, candidate.observation().timestampSeconds());
    }
    return maximum;
  }

  private static double sumLinearStandardDeviations(List<Candidate> candidates) {
    double sum = 0.0;
    for (Candidate candidate : candidates) {
      sum += candidate.linearStdDevMeters();
    }
    return sum;
  }

  private static List<Integer> cameraIndices(List<Candidate> candidates) {
    return candidates.stream()
        .map(Candidate::cameraIndex)
        .sorted(Comparator.naturalOrder())
        .toList();
  }
}
