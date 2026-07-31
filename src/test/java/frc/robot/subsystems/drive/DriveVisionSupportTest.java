package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.Optional;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DriveVisionSupportTest {
  private static final double EPSILON = 1e-9;

  private final SampledGyroIO gyroIO = new SampledGyroIO();
  private final SampledModuleIO frontLeft =
      new SampledModuleIO(TunerConstants.FrontLeft.WheelRadius);
  private final SampledModuleIO frontRight =
      new SampledModuleIO(TunerConstants.FrontRight.WheelRadius);
  private final SampledModuleIO backLeft = new SampledModuleIO(TunerConstants.BackLeft.WheelRadius);
  private final SampledModuleIO backRight =
      new SampledModuleIO(TunerConstants.BackRight.WheelRadius);
  private Drive drive;

  @BeforeAll
  void initializeDrive() {
    HAL.initialize(500, 0);
    drive = new Drive(gyroIO, frontLeft, frontRight, backLeft, backRight);
  }

  @BeforeEach
  void resetDriveState() {
    gyroIO.setTilt(0.0, 0.0);
    gyroIO.clearOdometrySamples();
    for (SampledModuleIO module : modules()) {
      module.clearOdometrySamples();
    }
    drive.setPoseAndSimulationTruth(Pose2d.kZero);
  }

  @Test
  void batchedTimestampedSamplesAdvanceTruthForEveryIndexedSample() {
    assertTrue(drive.samplePoseAt(5.0).isEmpty());
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Drive.getModuleTranslations());
    SwerveModulePosition[] initialPositions = currentModulePositions();
    SwerveModulePosition[] firstPositions =
        offsetModulePositions(
            initialPositions,
            new double[] {0.8, 1.0, 0.7, 1.1},
            new double[] {20.0, 35.0, 10.0, 45.0});
    SwerveModulePosition[] secondPositions =
        offsetModulePositions(
            initialPositions,
            new double[] {1.8, 2.2, 1.5, 2.4},
            new double[] {80.0, 100.0, 65.0, 120.0});
    Rotation2d firstYaw = Rotation2d.fromDegrees(15.0);
    Rotation2d secondYaw = Rotation2d.fromDegrees(40.0);
    SwerveDriveOdometry expectedTruth =
        new SwerveDriveOdometry(kinematics, Rotation2d.kZero, initialPositions, Pose2d.kZero);
    expectedTruth.update(firstYaw, firstPositions);
    Pose2d expectedAtFirstTimestamp = expectedTruth.getPoseMeters();
    expectedTruth.update(secondYaw, secondPositions);
    Pose2d expectedFinalPose = expectedTruth.getPoseMeters();

    SwerveDriveOdometry finalOnlyMutation =
        new SwerveDriveOdometry(kinematics, Rotation2d.kZero, initialPositions, Pose2d.kZero);
    finalOnlyMutation.update(secondYaw, secondPositions);
    SwerveDriveOdometry reusedFinalPositionsMutation =
        new SwerveDriveOdometry(kinematics, Rotation2d.kZero, initialPositions, Pose2d.kZero);
    reusedFinalPositionsMutation.update(firstYaw, secondPositions);
    reusedFinalPositionsMutation.update(secondYaw, secondPositions);
    assertPosesDiffer(expectedFinalPose, finalOnlyMutation.getPoseMeters());
    assertPosesDiffer(expectedFinalPose, reusedFinalPositionsMutation.getPoseMeters());

    setOdometrySamples(
        new double[] {5.0, 6.0},
        new Rotation2d[] {firstYaw, secondYaw},
        new SwerveModulePosition[][] {firstPositions, secondPositions});
    drive.periodic();

    Optional<Pose2d> sampledPose = drive.samplePoseAt(5.0);
    assertTrue(sampledPose.isPresent());
    assertPoseEquals(expectedAtFirstTimestamp, sampledPose.orElseThrow());
    assertPoseEquals(expectedFinalPose, drive.getSimulationPose());
  }

  @Test
  void pitchRollStabilityUsesInclusiveAbsoluteLimitsForBothAxes() {
    assertTiltCase(10.0, 8.0, 10.0, true);
    assertTiltCase(-10.0, -8.0, 10.0, true);
    assertTiltCase(8.0, 10.0, 10.0, true);
    assertTiltCase(-8.0, -10.0, 10.0, true);
    assertTiltCase(10.0001, 0.0, 10.0, false);
    assertTiltCase(0.0, -10.0001, 10.0, false);
  }

  @Test
  void estimatorOnlyAndCombinedSimulationResetsHaveDistinctSemantics() {
    Pose2d estimatorOnlyPose = new Pose2d(2.0, -1.0, Rotation2d.fromDegrees(25.0));

    drive.setPose(estimatorOnlyPose);

    assertPoseEquals(estimatorOnlyPose, drive.getPose());
    assertPoseEquals(Pose2d.kZero, drive.getSimulationPose());

    Pose2d combinedPose = new Pose2d(-4.0, 3.0, Rotation2d.fromDegrees(-40.0));
    drive.setPoseAndSimulationTruth(combinedPose);

    assertPoseEquals(combinedPose, drive.getPose());
    assertPoseEquals(combinedPose, drive.getSimulationPose());
  }

  private void assertTiltCase(
      double pitchDegrees, double rollDegrees, double maxAbsTiltDegrees, boolean expectedStable) {
    gyroIO.setTilt(pitchDegrees, rollDegrees);
    drive.periodic();

    assertEquals(pitchDegrees, drive.getPitchDegrees(), EPSILON);
    assertEquals(rollDegrees, drive.getRollDegrees(), EPSILON);
    assertEquals(expectedStable, drive.isPitchRollStableForVision(maxAbsTiltDegrees));
  }

  private void setOdometrySamples(
      double[] timestamps, Rotation2d[] yawPositions, SwerveModulePosition[][] samplePositions) {
    gyroIO.setOdometrySamples(timestamps, yawPositions);
    SampledModuleIO[] modules = modules();
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      SwerveModulePosition[] moduleSamples = new SwerveModulePosition[samplePositions.length];
      for (int sampleIndex = 0; sampleIndex < samplePositions.length; sampleIndex++) {
        moduleSamples[sampleIndex] = samplePositions[sampleIndex][moduleIndex];
      }
      modules[moduleIndex].setOdometrySamples(timestamps, moduleSamples);
    }
  }

  private SwerveModulePosition[] currentModulePositions() {
    SampledModuleIO[] modules = modules();
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].currentPosition();
    }
    return positions;
  }

  private static SwerveModulePosition[] offsetModulePositions(
      SwerveModulePosition[] initialPositions,
      double[] distanceOffsetsMeters,
      double[] anglesDegrees) {
    SwerveModulePosition[] positions = new SwerveModulePosition[initialPositions.length];
    for (int i = 0; i < initialPositions.length; i++) {
      positions[i] =
          new SwerveModulePosition(
              initialPositions[i].distanceMeters + distanceOffsetsMeters[i],
              Rotation2d.fromDegrees(anglesDegrees[i]));
    }
    return positions;
  }

  private SampledModuleIO[] modules() {
    return new SampledModuleIO[] {frontLeft, frontRight, backLeft, backRight};
  }

  private static void assertPoseEquals(Pose2d expected, Pose2d actual) {
    assertEquals(expected.getX(), actual.getX(), EPSILON);
    assertEquals(expected.getY(), actual.getY(), EPSILON);
    assertEquals(expected.getRotation().getRadians(), actual.getRotation().getRadians(), EPSILON);
  }

  private static void assertPosesDiffer(Pose2d expected, Pose2d mutation) {
    assertTrue(expected.getTranslation().getDistance(mutation.getTranslation()) > 1e-3);
  }

  private static final class SampledGyroIO implements GyroIO {
    private double pitchDegrees;
    private double rollDegrees;
    private double[] odometryTimestamps = new double[] {};
    private Rotation2d[] odometryPositions = new Rotation2d[] {};

    void setTilt(double pitchDegrees, double rollDegrees) {
      this.pitchDegrees = pitchDegrees;
      this.rollDegrees = rollDegrees;
    }

    void setOdometrySamples(double[] timestamps, Rotation2d[] yawPositions) {
      odometryTimestamps = timestamps.clone();
      odometryPositions = yawPositions.clone();
    }

    void clearOdometrySamples() {
      odometryTimestamps = new double[] {};
      odometryPositions = new Rotation2d[] {};
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.connected = true;
      inputs.pitchDegrees = pitchDegrees;
      inputs.rollDegrees = rollDegrees;
      inputs.odometryYawTimestamps = odometryTimestamps;
      inputs.odometryYawPositions = odometryPositions;
    }
  }

  private static final class SampledModuleIO implements ModuleIO {
    private final double wheelRadiusMeters;
    private double distanceMeters;
    private Rotation2d turnPosition = Rotation2d.kZero;
    private double[] odometryTimestamps = new double[] {};
    private double[] odometryDrivePositionsRad = new double[] {};
    private Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

    SampledModuleIO(double wheelRadiusMeters) {
      this.wheelRadiusMeters = wheelRadiusMeters;
    }

    void setOdometrySamples(double[] timestamps, SwerveModulePosition[] positions) {
      SwerveModulePosition finalPosition = positions[positions.length - 1];
      distanceMeters = finalPosition.distanceMeters;
      turnPosition = finalPosition.angle;
      odometryTimestamps = timestamps.clone();
      odometryDrivePositionsRad = new double[positions.length];
      odometryTurnPositions = new Rotation2d[positions.length];
      for (int i = 0; i < positions.length; i++) {
        odometryDrivePositionsRad[i] = positions[i].distanceMeters / wheelRadiusMeters;
        odometryTurnPositions[i] = positions[i].angle;
      }
    }

    SwerveModulePosition currentPosition() {
      return new SwerveModulePosition(distanceMeters, turnPosition);
    }

    void clearOdometrySamples() {
      odometryTimestamps = new double[] {};
      odometryDrivePositionsRad = new double[] {};
      odometryTurnPositions = new Rotation2d[] {};
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      inputs.driveConnected = true;
      inputs.turnConnected = true;
      inputs.turnEncoderConnected = true;
      inputs.drivePositionRad = distanceMeters / wheelRadiusMeters;
      inputs.turnPosition = turnPosition;
      inputs.odometryTimestamps = odometryTimestamps;
      inputs.odometryDrivePositionsRad = odometryDrivePositionsRad;
      inputs.odometryTurnPositions = odometryTurnPositions;
    }
  }
}
