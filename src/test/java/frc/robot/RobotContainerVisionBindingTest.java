package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionRuntimeConfig;
import frc.robot.util.constants.OperatorConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonCamera;

@Tag("vision-sim")
class RobotContainerVisionBindingTest {
  private static final Pose2d CAMERA_TRUTH_POSE =
      new Pose2d(4.407, 7.279, Rotation2d.fromDegrees(-90.0));

  @BeforeAll
  static void initializeHalOnce() {
    assertTrue(HAL.initialize(500, 0));
    PhotonCamera.setVersionCheckEnabled(false);
    SimHooks.pauseTiming();
  }

  @AfterAll
  static void restoreTiming() {
    SimHooks.resumeTiming();
  }

  @Test
  void operatorReseedIsEnabledOnlyWhileDisabledDriverResetRemainsAvailable() {
    var scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();

    var initializedCommands = new ArrayList<Command>();
    scheduler.onCommandInitialize(initializedCommands::add);

    var configCaptureCount = new AtomicInteger();
    var configuredDrive = new AtomicReference<Drive>();
    VisionRuntimeConfig expectedConfig = VisionRuntimeConfig.fromSystemProperties();
    var container =
        new RobotContainer(
            drive -> {
              configCaptureCount.incrementAndGet();
              configuredDrive.set(drive);
              return expectedConfig;
            });
    var driver = new XboxControllerSim(OperatorConstants.DRIVER_PORT);
    var operator = new XboxControllerSim(OperatorConstants.OPERATOR_PORT);
    var replacementTruthReads = new AtomicInteger();
    container.setVisionSimulationPoseSupplier(
        () -> {
          replacementTruthReads.incrementAndGet();
          return new Pose2d(-100.0, -100.0, Rotation2d.kZero);
        });

    assertNotNull(container.vision());
    assertEquals(1, configCaptureCount.get());
    assertSame(container.drive(), configuredDrive.get());
    var simulation = container.visionSimulation();
    assertNotNull(simulation);
    assertEquals(4, simulation.cameraCount());
    assertEquals(
        VisionConstants.CAMERAS.stream().map(config -> config.name()).toList(),
        simulation.cameraDiagnostics().stream().map(diagnostics -> diagnostics.name()).toList());
    assertEquals(
        VisionConstants.CAMERAS.stream().map(config -> config.robotToCamera()).toList(),
        simulation.cameraDiagnostics().stream()
            .map(diagnostics -> diagnostics.robotToCamera())
            .toList());
    assertEquals(OperatorConstants.DRIVER_PORT, container.driverControllerPort());
    assertEquals(OperatorConstants.OPERATOR_PORT, container.operatorControllerPort());
    assertNotEquals(container.driverControllerPort(), container.operatorControllerPort());

    Pose2d estimatorBeforeDriverReset = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(70.0));
    container.drive().setPose(estimatorBeforeDriverReset);
    Pose2d simulationTruthBeforeDriverReset = container.drive().getSimulationPose();
    setEnabled(false, scheduler);
    initializedCommands.clear();
    pressChord(driver, true, scheduler);
    Command driverReset = initializedNamed(initializedCommands, "Driver Heading Reset");
    assertEquals(Set.of(container.drive()), driverReset.getRequirements());
    assertTrue(driverReset.runsWhenDisabled());
    assertEquals(
        estimatorBeforeDriverReset.getTranslation(), container.drive().getPose().getTranslation());
    assertEquals(Rotation2d.kZero, container.drive().getPose().getRotation());
    assertEquals(simulationTruthBeforeDriverReset, container.drive().getSimulationPose());

    pressChord(driver, false, scheduler);
    initializedCommands.clear();
    pressChord(operator, true, scheduler);
    assertTrue(
        initializedCommands.stream()
            .noneMatch(command -> command.getName().equals("Force Vision Reseed")));

    pressChord(operator, false, scheduler);
    container.drive().setPose(CAMERA_TRUTH_POSE);
    container.setVisionSimulationPoseSupplier(
        () -> {
          replacementTruthReads.incrementAndGet();
          return CAMERA_TRUTH_POSE;
        });
    setEnabled(true, scheduler);
    var snapshot = waitForAcceptedSnapshot(container, scheduler);
    assertTrue(replacementTruthReads.get() > 0);

    Pose2d estimatorOffset =
        new Pose2d(
            snapshot.pose().getX() + 1.0,
            snapshot.pose().getY(),
            snapshot.pose().getRotation().plus(Rotation2d.fromDegrees(25.0)));
    container.drive().setPose(estimatorOffset);
    Pose2d simulationTruthBeforeOperatorReseed = container.drive().getSimulationPose();
    initializedCommands.clear();
    pressChord(operator, true, scheduler);
    Command forceReseed = initializedNamed(initializedCommands, "Force Vision Reseed");
    assertEquals(Set.of(container.vision()), forceReseed.getRequirements());
    assertFalse(forceReseed.runsWhenDisabled());
    assertPoseEquals(snapshot.pose(), container.drive().getPose());
    assertEquals(simulationTruthBeforeOperatorReseed, container.drive().getSimulationPose());

    pressChord(operator, false, scheduler);
    scheduler.cancelAll();
  }

  private static frc.robot.subsystems.vision.Vision.AcceptedObservationSnapshot
      waitForAcceptedSnapshot(RobotContainer container, CommandScheduler scheduler) {
    Optional<frc.robot.subsystems.vision.Vision.AcceptedObservationSnapshot> snapshot =
        Optional.empty();
    for (int cycle = 0; cycle < 200 && snapshot.isEmpty(); cycle++) {
      SimHooks.stepTiming(0.020);
      DriverStationSim.notifyNewData();
      scheduler.run();
      snapshot = container.vision().getLatestAcceptedObservationSnapshot();
    }
    return snapshot.orElseThrow(
        () -> new AssertionError("SIM container produced no vision snapshot"));
  }

  private static Command initializedNamed(List<Command> commands, String name) {
    return commands.stream()
        .filter(command -> command.getName().equals(name))
        .findFirst()
        .orElseThrow(() -> new AssertionError("Command did not initialize: " + name));
  }

  private static void assertPoseEquals(Pose2d expected, Pose2d actual) {
    assertEquals(expected.getX(), actual.getX(), 1e-9);
    assertEquals(expected.getY(), actual.getY(), 1e-9);
    assertEquals(expected.getRotation().getRadians(), actual.getRotation().getRadians(), 1e-9);
  }

  private static void setEnabled(boolean enabled, CommandScheduler scheduler) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
    scheduler.run();
  }

  private static void pressChord(
      XboxControllerSim controller, boolean pressed, CommandScheduler scheduler) {
    controller.setStartButton(pressed);
    controller.setBackButton(pressed);
    DriverStationSim.notifyNewData();
    scheduler.run();
  }
}
