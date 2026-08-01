// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

public class HubShiftUtil {
  public enum ShiftEnum {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED;
  }

  public record ShiftInfo(
      ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

  private static Timer shiftTimer = new Timer();
  private static final ShiftEnum[] SHIFT_ENUMS = ShiftEnum.values();

  private static final double[] SHIFT_START_TIMES = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] SHIFT_END_TIMES = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  private static final double APPROACHING_ACTIVE_FUDGE_SECONDS = -2.0;
  private static final double ENDING_ACTIVE_FUDGE_SECONDS = -0.5;

  public static final double autoEndTime = 20.0;
  public static final double teleopDuration = 140.0;
  private static final boolean[] STARTING_ACTIVE = {true, true, false, true, false, true};
  private static final boolean[] STARTING_INACTIVE = {true, false, true, false, true, true};
  private static final double TIME_RESET_THRESHOLD_SECONDS = 3.0;
  private static double shiftTimerOffset = 0.0;
  private static Supplier<Optional<Boolean>> allianceWinOverride = Optional::empty;

  public static void setAllianceWinOverride(Supplier<Optional<Boolean>> allianceWinOverride) {
    HubShiftUtil.allianceWinOverride = Objects.requireNonNull(allianceWinOverride);
  }

  public static Optional<Boolean> getAllianceWinOverride() {
    return allianceWinOverride.get();
  }

  public static Alliance getFirstActiveAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    var winOverride = getAllianceWinOverride();
    if (!winOverride.isEmpty()) {
      return winOverride.get()
          ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
          : (alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red);
    }

    String message = DriverStation.getGameSpecificMessage();
    if (!message.isEmpty()) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Blue;
      } else if (character == 'B') {
        return Alliance.Red;
      }
    }

    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  /** Starts the timer at the beginning of teleop. */
  public static void initialize() {
    shiftTimerOffset = 0.0;
    shiftTimer.restart();
  }

  private static boolean[] getSchedule() {
    Alliance startAlliance = getFirstActiveAlliance();
    return startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
        ? STARTING_ACTIVE
        : STARTING_INACTIVE;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {
    double timerValue = shiftTimer.get();
    double currentTime = timerValue - shiftTimerOffset;
    double stateTimeElapsed = currentTime;
    double stateTimeRemaining = 0.0;
    boolean active = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;
    double fieldTeleopTime = teleopDuration - DriverStation.getMatchTime();

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = autoEndTime - currentTime;
      active = true;
      currentShift = ShiftEnum.AUTO;
    } else if (DriverStation.isEnabled()) {
      if (Math.abs(fieldTeleopTime - currentTime) >= TIME_RESET_THRESHOLD_SECONDS
          && fieldTeleopTime <= 135.0
          && DriverStation.isFMSAttached()) {
        shiftTimerOffset += currentTime - fieldTeleopTime;
        currentTime = timerValue - shiftTimerOffset;
      }
      int currentShiftIndex = -1;
      for (int i = 0; i < shiftStartTimes.length; i++) {
        if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
          currentShiftIndex = i;
          break;
        }
      }
      if (currentShiftIndex < 0) {
        currentShiftIndex = shiftStartTimes.length - 1;
      }

      stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
      stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

      if (currentShiftIndex > 0
          && currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
        stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
      }

      if (currentShiftIndex < shiftEndTimes.length - 1
          && currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
        stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
      }

      active = currentSchedule[currentShiftIndex];
      currentShift = SHIFT_ENUMS[currentShiftIndex];
    }
    return new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), SHIFT_START_TIMES, SHIFT_END_TIMES);
  }

  public static ShiftInfo getShiftedShiftInfo() {
    boolean[] shiftSchedule = getSchedule();
    if (shiftSchedule[1]) {
      double[] shiftedShiftStartTimes = {
        0.0,
        10.0,
        35.0 + ENDING_ACTIVE_FUDGE_SECONDS,
        60.0 + APPROACHING_ACTIVE_FUDGE_SECONDS,
        85.0 + ENDING_ACTIVE_FUDGE_SECONDS,
        110.0 + APPROACHING_ACTIVE_FUDGE_SECONDS
      };
      double[] shiftedShiftEndTimes = {
        10.0,
        35.0 + ENDING_ACTIVE_FUDGE_SECONDS,
        60.0 + APPROACHING_ACTIVE_FUDGE_SECONDS,
        85.0 + ENDING_ACTIVE_FUDGE_SECONDS,
        110.0 + APPROACHING_ACTIVE_FUDGE_SECONDS,
        140.0
      };
      return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
    }
    double[] shiftedShiftStartTimes = {
      0.0,
      10.0 + ENDING_ACTIVE_FUDGE_SECONDS,
      35.0 + APPROACHING_ACTIVE_FUDGE_SECONDS,
      60.0 + ENDING_ACTIVE_FUDGE_SECONDS,
      85.0 + APPROACHING_ACTIVE_FUDGE_SECONDS,
      110.0
    };
    double[] shiftedShiftEndTimes = {
      10.0 + ENDING_ACTIVE_FUDGE_SECONDS,
      35.0 + APPROACHING_ACTIVE_FUDGE_SECONDS,
      60.0 + ENDING_ACTIVE_FUDGE_SECONDS,
      85.0 + APPROACHING_ACTIVE_FUDGE_SECONDS,
      110.0,
      140.0
    };
    return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
  }
}
