package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.junit.jupiter.api.Test;

class DriveSimulationPoseTrackerTest {
  private static final double EPSILON = 1e-9;
  private static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(0.5, 0.5),
          new Translation2d(0.5, -0.5),
          new Translation2d(-0.5, 0.5),
          new Translation2d(-0.5, -0.5));

  @Test
  void updateTracksKnownForwardModuleMovement() {
    DriveSimulationPoseTracker tracker = createTracker();

    tracker.update(Rotation2d.kZero, modulePositions(1.25));

    assertPoseEquals(new Pose2d(1.25, 0.0, Rotation2d.kZero), tracker.getPose());
  }

  @Test
  void estimatorResetDoesNotMoveIndependentSimulationTruth() {
    SwerveModulePosition[] initialPositions = modulePositions(0.0);
    DriveSimulationPoseTracker tracker = createTracker();
    SwerveDrivePoseEstimator estimator =
        new SwerveDrivePoseEstimator(KINEMATICS, Rotation2d.kZero, initialPositions, Pose2d.kZero);
    tracker.update(Rotation2d.kZero, modulePositions(1.0));
    Pose2d estimatorResetPose = new Pose2d(4.0, -2.0, Rotation2d.fromDegrees(35.0));

    estimator.resetPosition(Rotation2d.kZero, modulePositions(1.0), estimatorResetPose);

    assertPoseEquals(estimatorResetPose, estimator.getEstimatedPosition());
    assertPoseEquals(new Pose2d(1.0, 0.0, Rotation2d.kZero), tracker.getPose());
  }

  @Test
  void trackerResetMovesSimulationTruthToRequestedPose() {
    DriveSimulationPoseTracker tracker = createTracker();
    tracker.update(Rotation2d.kZero, modulePositions(1.0));
    Pose2d requestedPose = new Pose2d(-3.0, 2.5, Rotation2d.fromDegrees(-70.0));

    tracker.resetPosition(Rotation2d.kZero, modulePositions(1.0), requestedPose);

    assertPoseEquals(requestedPose, tracker.getPose());
  }

  private static DriveSimulationPoseTracker createTracker() {
    return new DriveSimulationPoseTracker(
        KINEMATICS, Rotation2d.kZero, modulePositions(0.0), Pose2d.kZero);
  }

  private static SwerveModulePosition[] modulePositions(double distanceMeters) {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(distanceMeters, Rotation2d.kZero),
      new SwerveModulePosition(distanceMeters, Rotation2d.kZero),
      new SwerveModulePosition(distanceMeters, Rotation2d.kZero),
      new SwerveModulePosition(distanceMeters, Rotation2d.kZero)
    };
  }

  private static void assertPoseEquals(Pose2d expected, Pose2d actual) {
    assertEquals(expected.getX(), actual.getX(), EPSILON);
    assertEquals(expected.getY(), actual.getY(), EPSILON);
    assertEquals(expected.getRotation().getRadians(), actual.getRotation().getRadians(), EPSILON);
  }
}
