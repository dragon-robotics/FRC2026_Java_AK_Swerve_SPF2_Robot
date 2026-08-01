package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVision.HeadingProvider;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionRuntimeConfig;
import frc.robot.subsystems.vision.VisionSimulation;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class RobotContainerVisionTest {
  @Test
  void constructsDriveBeforeCapturingConfigOnceAndBuildingVision() {
    var events = new ArrayList<String>();
    var expectedConfig = VisionRuntimeConfig.fromSystemProperties();

    var construction =
        RobotContainer.constructDriveThenVision(
            () -> {
              events.add("drive");
              return "constructed-drive";
            },
            () -> {
              events.add("config");
              return expectedConfig;
            },
            (drive, config) -> {
              events.add("vision");
              assertEquals("constructed-drive", drive);
              assertSame(expectedConfig, config);
              return "constructed-vision";
            });

    assertEquals(List.of("drive", "config", "vision"), events);
    assertEquals("constructed-drive", construction.drive());
    assertEquals("constructed-vision", construction.vision());
  }

  @Test
  void realFactoriesNameFourPhotonAdaptersWithoutConstructingThem() {
    var factories =
        RobotContainer.visionIOFactories(
            VisionConstructionPlan.forMode(Mode.REAL),
            VisionRuntimeConfig.fromSystemProperties(),
            unusedHeadingProvider(),
            null);

    assertEquals(4, factories.size());
    assertEquals(
        List.of(
            "AprilTagPoseEstCameraF",
            "AprilTagPoseEstCameraR",
            "AprilTagPoseEstCameraB",
            "AprilTagPoseEstCameraL"),
        factories.stream().map(RobotContainer.VisionIOFactory::cameraName).toList());
    assertTrue(
        factories.stream()
            .allMatch(factory -> factory.implementationType() == VisionIOPhotonVision.class));
  }

  @Test
  void simFactoriesStayDeferredAndRetainOneSharedOwner() {
    try (var simulation = new VisionSimulation("robot-container-factory-test", Pose2d::new)) {
      var factories =
          RobotContainer.visionIOFactories(
              VisionConstructionPlan.forMode(Mode.SIM),
              VisionRuntimeConfig.fromSystemProperties(),
              unusedHeadingProvider(),
              simulation);

      assertEquals(4, factories.size());
      assertTrue(
          factories.stream()
              .allMatch(factory -> factory.implementationType() == VisionIOPhotonVisionSim.class));
      assertTrue(factories.stream().allMatch(factory -> factory.simulationOwner() == simulation));
      assertEquals(
          0, simulation.cameraCount(), "deferred factories must not create Photon cameras");
    }
  }

  @Test
  void replayFactoriesCreateFourNamedNoOps() {
    var factories =
        RobotContainer.visionIOFactories(
            VisionConstructionPlan.forMode(Mode.REPLAY),
            VisionRuntimeConfig.fromSystemProperties(),
            unusedHeadingProvider(),
            null);

    List<VisionIO> io = factories.stream().map(RobotContainer.VisionIOFactory::create).toList();

    assertTrue(io.stream().allMatch(VisionIO.NoOp.class::isInstance));
    assertEquals(
        List.of(
            "AprilTagPoseEstCameraF",
            "AprilTagPoseEstCameraR",
            "AprilTagPoseEstCameraB",
            "AprilTagPoseEstCameraL"),
        io.stream().map(VisionIO::getCameraName).toList());
  }

  @Test
  void headingProviderUsesTimestampedPoseAndMeasuredSpeeds() {
    var sampledTimestamp = new AtomicReference<Double>();
    Pose2d sampledPose = new Pose2d(new Translation2d(2.25, 7.5), Rotation2d.fromDegrees(37.0));
    ChassisSpeeds measuredSpeeds = new ChassisSpeeds(3.0, 4.0, -1.75);
    HeadingProvider provider =
        RobotContainer.visionHeadingProvider(
            timestamp -> {
              sampledTimestamp.set(timestamp);
              return Optional.of(sampledPose);
            },
            () -> measuredSpeeds);

    assertEquals(Optional.of(sampledPose.getRotation()), provider.headingAt(12.5));
    assertEquals(12.5, sampledTimestamp.get());
    assertEquals(Optional.of(new Pose3d(sampledPose)), provider.seedPoseAt(12.75));
    assertEquals(12.75, sampledTimestamp.get());
    assertEquals(-1.75, provider.angularRateRadPerSecond());
    assertEquals(5.0, provider.linearSpeedMetersPerSecond());
  }

  @Test
  void simUsesOwnerUpdateAsThePreInputHookOnlyInSim() {
    var updateCount = new AtomicInteger();

    RobotContainer.visionPreInputHook(Mode.SIM, updateCount::incrementAndGet).run();
    RobotContainer.visionPreInputHook(Mode.REAL, () -> fail("REAL must not update sim")).run();
    RobotContainer.visionPreInputHook(Mode.REPLAY, () -> fail("REPLAY must not update sim")).run();

    assertEquals(1, updateCount.get());
  }

  @Test
  void startupExplicitlySetsAimingFalse() {
    var aiming = new AtomicReference<Boolean>();

    RobotContainer.initializeVisionAiming(aiming::set);

    assertEquals(Boolean.FALSE, aiming.get());
  }

  @Test
  void simPoseSupplierOverrideReplacesOwnerTruthAndIsNoOpWithoutOwner() {
    var replacementCalls = new AtomicInteger();
    Pose2d replacementPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(15.0));
    try (var simulation = new VisionSimulation("robot-container-pose-test", Pose2d::new)) {
      RobotContainer.setVisionSimulationPoseSupplier(
          simulation,
          () -> {
            replacementCalls.incrementAndGet();
            return replacementPose;
          });

      simulation.update();
      assertEquals(1, replacementCalls.get());
    }

    assertDoesNotThrow(
        () ->
            RobotContainer.setVisionSimulationPoseSupplier(
                null, () -> fail("non-SIM modes must not read the replacement supplier")));
  }

  private static HeadingProvider unusedHeadingProvider() {
    return new HeadingProvider() {
      @Override
      public Optional<Rotation2d> headingAt(double fpgaTimestampSeconds) {
        return Optional.empty();
      }

      @Override
      public Optional<Pose3d> seedPoseAt(double fpgaTimestampSeconds) {
        return Optional.empty();
      }

      @Override
      public double angularRateRadPerSecond() {
        return 0.0;
      }

      @Override
      public double linearSpeedMetersPerSecond() {
        return 0.0;
      }
    };
  }
}
