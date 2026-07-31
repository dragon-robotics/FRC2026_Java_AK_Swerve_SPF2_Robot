package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  void timestampedSamplesUseEstimatorFpgaSecondsWithoutConversion() {
    assertTrue(drive.samplePoseAt(5.5).isEmpty());
    double initialDistanceMeters = frontLeft.distanceMeters;

    setOdometrySample(5.0, initialDistanceMeters + 1.0);
    drive.periodic();
    setOdometrySample(6.0, initialDistanceMeters + 3.0);
    drive.periodic();

    Optional<Pose2d> sampledPose = drive.samplePoseAt(5.5);
    assertTrue(sampledPose.isPresent());
    assertEquals(2.0, sampledPose.orElseThrow().getX(), EPSILON);
    assertEquals(0.0, sampledPose.orElseThrow().getY(), EPSILON);
    assertPoseEquals(new Pose2d(3.0, 0.0, Rotation2d.kZero), drive.getSimulationPose());
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

  private void setOdometrySample(double timestampSeconds, double distanceMeters) {
    gyroIO.setOdometrySample(timestampSeconds, Rotation2d.kZero);
    for (SampledModuleIO module : modules()) {
      module.setOdometrySample(timestampSeconds, distanceMeters, Rotation2d.kZero);
    }
  }

  private SampledModuleIO[] modules() {
    return new SampledModuleIO[] {frontLeft, frontRight, backLeft, backRight};
  }

  private static void assertPoseEquals(Pose2d expected, Pose2d actual) {
    assertEquals(expected.getX(), actual.getX(), EPSILON);
    assertEquals(expected.getY(), actual.getY(), EPSILON);
    assertEquals(expected.getRotation().getRadians(), actual.getRotation().getRadians(), EPSILON);
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

    void setOdometrySample(double timestampSeconds, Rotation2d yawPosition) {
      odometryTimestamps = new double[] {timestampSeconds};
      odometryPositions = new Rotation2d[] {yawPosition};
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

    void setOdometrySample(
        double timestampSeconds, double distanceMeters, Rotation2d turnPosition) {
      this.distanceMeters = distanceMeters;
      this.turnPosition = turnPosition;
      odometryTimestamps = new double[] {timestampSeconds};
      odometryDrivePositionsRad = new double[] {distanceMeters / wheelRadiusMeters};
      odometryTurnPositions = new Rotation2d[] {turnPosition};
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
