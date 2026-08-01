package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.util.constants.FieldConstants;
import java.lang.reflect.Field;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

class SuperstructureLoggingTest {
  private SuperstructureTestHarness harness;

  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    harness = new SuperstructureTestHarness();
  }

  @AfterEach
  void tearDown() {
    harness.close();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  @Test
  void periodicPublishesAndClearsTheCoordinationContract() throws ReflectiveOperationException {
    try (LoggerCapture capture = LoggerCapture.start()) {
      harness.setAlliance(AllianceStationID.Red1);
      harness.superstructure.periodic();
      assertEquals(
          "DRIVE_STARTING_CONFIG",
          capture.table().get("RealOutputs/Superstructure/CurrentState", ""));
      assertTrue(capture.table().get("RealOutputs/Superstructure/AllianceConfirmed", false));
      assertFalse(capture.table().get("RealOutputs/Superstructure/Zone", "").isBlank());
      assertEquals(
          FieldConstants.Hub.RED_CENTER_POSE, harness.superstructure.getCurrentAimTarget());

      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
      harness.superstructure.periodic();
      assertFalse(capture.table().get("RealOutputs/Superstructure/AllianceConfirmed", true));
      assertEquals("UNCONFIRMED", capture.table().get("RealOutputs/Superstructure/Alliance", ""));
      assertEquals("UNCONFIRMED", capture.table().get("RealOutputs/Superstructure/Zone", ""));
      assertEquals(
          FieldConstants.Hub.BLUE_CENTER_POSE, harness.superstructure.getCurrentAimTarget());

      Set<String> requiredKeys =
          Set.of(
              "RealOutputs/Superstructure/CurrentState",
              "RealOutputs/Superstructure/ShootMode",
              "RealOutputs/Superstructure/AllianceConfirmed",
              "RealOutputs/Superstructure/Alliance",
              "RealOutputs/Superstructure/Zone",
              "RealOutputs/Superstructure/AimTarget",
              "RealOutputs/Superstructure/DistanceToTargetMeters",
              "RealOutputs/Superstructure/DistanceToTargetFeet",
              "RealOutputs/Superstructure/IsAlignedToTarget",
              "RealOutputs/Superstructure/ShooterReady",
              "RealOutputs/Superstructure/FeedReady",
              "RealOutputs/Superstructure/ShootAllowed",
              "RealOutputs/Superstructure/PurgeZone",
              "RealOutputs/HubShift/Official/CurrentShift",
              "RealOutputs/HubShift/Official/Active",
              "RealOutputs/HubShift/Official/ElapsedTime",
              "RealOutputs/HubShift/Official/RemainingTime",
              "RealOutputs/HubShift/Shifted/CurrentShift",
              "RealOutputs/HubShift/Shifted/Active",
              "RealOutputs/HubShift/Shifted/ElapsedTime",
              "RealOutputs/HubShift/Shifted/RemainingTime",
              "RealOutputs/HubShift/FirstActiveAlliance");
      Set<String> actualKeys =
          capture.table().getAll(false).keySet().stream()
              .map(key -> key.startsWith("/") ? key.substring(1) : key)
              .collect(java.util.stream.Collectors.toSet());
      assertTrue(
          actualKeys.containsAll(requiredKeys),
          () ->
              "Missing coordination keys: "
                  + requiredKeys.stream().filter(key -> !actualKeys.contains(key)).toList()
                  + "; actual keys: "
                  + actualKeys);
    }
  }

  static final class LoggerCapture implements AutoCloseable {
    private final Field runningField;
    private final Field entryField;
    private final Field outputTableField;
    private final boolean previousRunning;
    private final LogTable previousEntry;
    private final LogTable previousOutputTable;
    private final LogTable table;

    private LoggerCapture() throws ReflectiveOperationException {
      runningField = Logger.class.getDeclaredField("running");
      entryField = Logger.class.getDeclaredField("entry");
      outputTableField = Logger.class.getDeclaredField("outputTable");
      runningField.setAccessible(true);
      entryField.setAccessible(true);
      outputTableField.setAccessible(true);
      previousRunning = runningField.getBoolean(null);
      previousEntry = (LogTable) entryField.get(null);
      previousOutputTable = (LogTable) outputTableField.get(null);
      table = new LogTable(0);
      runningField.setBoolean(null, true);
      entryField.set(null, table);
      outputTableField.set(null, table.getSubtable("RealOutputs"));
    }

    static LoggerCapture start() throws ReflectiveOperationException {
      return new LoggerCapture();
    }

    LogTable table() {
      return table;
    }

    @Override
    public void close() throws ReflectiveOperationException {
      outputTableField.set(null, previousOutputTable);
      entryField.set(null, previousEntry);
      runningField.setBoolean(null, previousRunning);
    }
  }
}
