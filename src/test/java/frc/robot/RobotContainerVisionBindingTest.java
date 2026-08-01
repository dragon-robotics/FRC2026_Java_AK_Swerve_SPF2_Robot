package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.constants.OperatorConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("vision-sim")
class RobotContainerVisionBindingTest {
  @BeforeAll
  static void initializeHalOnce() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void operatorReseedIsEnabledOnlyWhileDisabledDriverResetRemainsAvailable() {
    var scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();

    var initializedCommands = new ArrayList<String>();
    scheduler.onCommandInitialize(command -> initializedCommands.add(command.getName()));

    var container = new RobotContainer();
    var driver = new XboxControllerSim(OperatorConstants.DRIVER_PORT);
    var operator = new XboxControllerSim(OperatorConstants.OPERATOR_PORT);
    var replacementTruthReads = new AtomicInteger();
    container.setVisionSimulationPoseSupplier(
        () -> {
          replacementTruthReads.incrementAndGet();
          return Pose2d.kZero;
        });

    assertNotNull(container.vision());
    assertEquals(OperatorConstants.DRIVER_PORT, container.driverControllerPort());
    assertEquals(OperatorConstants.OPERATOR_PORT, container.operatorControllerPort());
    assertNotEquals(container.driverControllerPort(), container.operatorControllerPort());

    setEnabled(true, scheduler);
    assertTrue(replacementTruthReads.get() > 0);
    pressChord(operator, true, scheduler);
    assertTrue(initializedCommands.contains("Force Vision Reseed"));

    pressChord(operator, false, scheduler);
    initializedCommands.clear();
    setEnabled(false, scheduler);
    pressChord(operator, true, scheduler);
    assertFalse(initializedCommands.contains("Force Vision Reseed"));

    pressChord(operator, false, scheduler);
    initializedCommands.clear();
    pressChord(driver, true, scheduler);
    assertEquals(List.of("Driver Heading Reset"), initializedCommands);

    pressChord(driver, false, scheduler);
    scheduler.cancelAll();
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
