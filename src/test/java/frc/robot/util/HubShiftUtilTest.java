package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.util.HubShiftUtil.ShiftEnum;
import frc.robot.util.HubShiftUtil.ShiftInfo;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class HubShiftUtilTest {
  private static final double EPSILON = 1e-9;

  private Timer originalTimer;
  private final FixedTimer fixedTimer = new FixedTimer();

  @BeforeEach
  void setUp() throws Exception {
    HAL.initialize(500, 0);
    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.notifyNewData();

    HubShiftUtil.setAllianceWinOverride(() -> Optional.of(false));

    originalTimer = getTimer();
    setTimer(fixedTimer);
    setShiftTimerOffset(0.0);
  }

  @AfterEach
  void tearDown() throws Exception {
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();

    HubShiftUtil.setAllianceWinOverride(Optional::empty);
    setTimer(originalTimer);
    setShiftTimerOffset(0.0);
  }

  @Test
  void officialShiftStartRemainingTargetsMatchCombinedTiming() {
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.notifyNewData();
    DriverStationSim.setAutonomous(true);
    fixedTimer.setTime(0.0);
    DriverStationSim.notifyNewData();
    ShiftInfo autoInfo = HubShiftUtil.getOfficialShiftInfo();
    assertEquals(ShiftEnum.AUTO, autoInfo.currentShift());

    DriverStationSim.setAutonomous(false);
    ShiftCheckpoint[] checkpoints = {
      new ShiftCheckpoint(5.0, 140.0, ShiftEnum.TRANSITION, 35.0),
      new ShiftCheckpoint(10.0, 130.0, ShiftEnum.SHIFT1, 25.0),
      new ShiftCheckpoint(35.0, 105.0, ShiftEnum.SHIFT2, 25.0),
      new ShiftCheckpoint(60.0, 80.0, ShiftEnum.SHIFT3, 25.0),
      new ShiftCheckpoint(85.0, 55.0, ShiftEnum.SHIFT4, 25.0),
      new ShiftCheckpoint(110.0, 30.0, ShiftEnum.ENDGAME, 30.0)
    };

    List<String> mismatches = new ArrayList<>();
    for (ShiftCheckpoint checkpoint : checkpoints) {
      fixedTimer.setTime(checkpoint.timerSeconds());
      DriverStationSim.setMatchTime(checkpoint.matchTimeSeconds());
      DriverStationSim.notifyNewData();

      ShiftInfo info = HubShiftUtil.getOfficialShiftInfo();
      assertEquals(checkpoint.expectedShift(), info.currentShift());
      if (Math.abs(info.remainingTime() - checkpoint.expectedRemainingSeconds()) > EPSILON) {
        mismatches.add(
            String.format(
                "%s expectedRemaining=%.2f actualRemaining=%.2f",
                checkpoint.expectedShift(),
                checkpoint.expectedRemainingSeconds(),
                info.remainingTime()));
      }
    }

    assertTrue(
        mismatches.isEmpty(), "Expected no remaining-time mismatches, but found: " + mismatches);
  }

  @Test
  void shiftedScheduleOpensTwoSecondsEarlyAndClosesHalfSecondEarly() {
    HubShiftUtil.setAllianceWinOverride(() -> Optional.of(false));

    fixedTimer.setTime(34.49);
    assertTrue(HubShiftUtil.getShiftedShiftInfo().active());

    fixedTimer.setTime(34.50);
    assertFalse(HubShiftUtil.getShiftedShiftInfo().active());

    fixedTimer.setTime(57.99);
    assertFalse(HubShiftUtil.getShiftedShiftInfo().active());

    fixedTimer.setTime(58.00);
    assertTrue(HubShiftUtil.getShiftedShiftInfo().active());
  }

  @Test
  void autoIsAlwaysActiveAndDisabledIsInactive() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    assertEquals(ShiftEnum.AUTO, HubShiftUtil.getOfficialShiftInfo().currentShift());
    assertTrue(HubShiftUtil.getOfficialShiftInfo().active());

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    assertEquals(ShiftEnum.DISABLED, HubShiftUtil.getOfficialShiftInfo().currentShift());
    assertFalse(HubShiftUtil.getOfficialShiftInfo().active());
  }

  @Test
  void allianceMessageAndOverrideSelectTheFirstActiveAlliance() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setGameSpecificMessage("R");
    DriverStationSim.notifyNewData();
    HubShiftUtil.setAllianceWinOverride(Optional::empty);
    assertEquals(Alliance.Blue, HubShiftUtil.getFirstActiveAlliance());

    DriverStationSim.setGameSpecificMessage("B");
    DriverStationSim.notifyNewData();
    assertEquals(Alliance.Red, HubShiftUtil.getFirstActiveAlliance());

    HubShiftUtil.setAllianceWinOverride(() -> Optional.of(false));
    assertEquals(Alliance.Blue, HubShiftUtil.getFirstActiveAlliance());
    HubShiftUtil.setAllianceWinOverride(() -> Optional.of(true));
    assertEquals(Alliance.Red, HubShiftUtil.getFirstActiveAlliance());

    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    HubShiftUtil.setAllianceWinOverride(() -> Optional.of(false));
    assertEquals(Alliance.Red, HubShiftUtil.getFirstActiveAlliance());
    HubShiftUtil.setAllianceWinOverride(() -> Optional.of(true));
    assertEquals(Alliance.Blue, HubShiftUtil.getFirstActiveAlliance());
  }

  @Test
  void fmsClockDifferenceOfAtLeastThreeSecondsResynchronizes() {
    HubShiftUtil.setAllianceWinOverride(() -> Optional.of(false));
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setMatchTime(100.0);
    DriverStationSim.notifyNewData();
    fixedTimer.setTime(20.0);

    ShiftInfo info = HubShiftUtil.getOfficialShiftInfo();

    assertEquals(ShiftEnum.SHIFT2, info.currentShift());
    assertEquals(5.0, info.elapsedTime(), 1e-9);
    assertEquals(20.0, info.remainingTime(), 1e-9);
  }

  private record ShiftCheckpoint(
      double timerSeconds,
      double matchTimeSeconds,
      ShiftEnum expectedShift,
      double expectedRemainingSeconds) {}

  private static Timer getTimer() throws Exception {
    Field timerField = HubShiftUtil.class.getDeclaredField("shiftTimer");
    timerField.setAccessible(true);
    return (Timer) timerField.get(null);
  }

  private static void setTimer(Timer timer) throws Exception {
    Field timerField = HubShiftUtil.class.getDeclaredField("shiftTimer");
    timerField.setAccessible(true);
    timerField.set(null, timer);
  }

  private static void setShiftTimerOffset(double offset) throws Exception {
    Field offsetField = HubShiftUtil.class.getDeclaredField("shiftTimerOffset");
    offsetField.setAccessible(true);
    offsetField.setDouble(null, offset);
  }

  private static class FixedTimer extends Timer {
    private double time;

    void setTime(double time) {
      this.time = time;
    }

    @Override
    public double get() {
      return time;
    }

    @Override
    public void restart() {
      // No-op for deterministic tests.
    }
  }
}
