package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.HubShiftUtil;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RobotHubShiftLifecycleTest {
  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    HubShiftUtil.setAllianceWinOverride(Optional::empty);
  }

  @Test
  void lifecycleHelpersInitializeBothModesAndCancelTheAutoCommand() {
    AtomicInteger calls = new AtomicInteger();
    Command auto = Commands.idle().ignoringDisable(true);
    Command scheduled = Robot.startAutonomous(calls::incrementAndGet, () -> auto);
    assertSame(auto, scheduled);
    assertTrue(auto.isScheduled());

    Robot.startTeleop(calls::incrementAndGet, scheduled);

    assertEquals(2, calls.get());
    assertFalse(auto.isScheduled());
  }

  @Test
  void dashboardOverrideParserIsTrimmedCaseInsensitiveAndSafe() {
    assertEquals(Optional.of(true), RobotContainer.parseHubShiftOverride(" true "));
    assertEquals(Optional.of(false), RobotContainer.parseHubShiftOverride("FaLsE"));
    assertEquals(Optional.empty(), RobotContainer.parseHubShiftOverride(""));
    assertEquals(Optional.empty(), RobotContainer.parseHubShiftOverride("winner"));
    assertEquals(Optional.empty(), RobotContainer.parseHubShiftOverride(null));
  }

  @Test
  void dashboardOverrideSupplierIsActuallyInstalled() {
    AtomicReference<Supplier<Optional<Boolean>>> installed = new AtomicReference<>();
    RobotContainer.installHubShiftOverride(installed::set, () -> " TrUe ");
    assertNotNull(installed.get());
    assertEquals(Optional.of(true), installed.get().get());
  }
}
