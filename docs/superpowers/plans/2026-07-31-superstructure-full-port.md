# Superstructure Full-Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the complete `spitfire-v2` superstructure, controller bindings, autonomous named commands, Hub Shift support, and AdvantageKit telemetry onto the existing target subsystem APIs.

**Architecture:** `Superstructure` is the command-based coordinator for Drive, Intake, Hopper, Shooter, and Vision, while a package-private pure targeting helper owns field geometry rules and `HubShiftUtil` owns game timing. Mechanism outputs remain in their existing subsystem state machines; Superstructure commands declare exact WPILib requirements, and `periodic()` performs target calculations, state updates, and logging only.

**Tech Stack:** Java 17, WPILib command framework and simulation, AdvantageKit, PathPlanner 2026.1.2, JUnit Jupiter 5.10.1, GradleRIO, Phoenix 6 simulation.

## Global Constraints

- Reference behavior is `FRC2026_Java_Swerve_Robot` `spitfire-v2` commit `70cce7cc0ee2c3b53644a582acf5b6bb9be35dd8`.
- Startup requests are Intake `HOME`, Hopper `STOP`, and Shooter `PREPFUEL`.
- Driver A holds `DRIVE_STARTING_CONFIG` and release explicitly enters `DRIVE`.
- Driver right-trigger default shooting is stationary.
- Automatic feed requires `ShooterState.SHOOT` and a strict heading error below 5 degrees.
- `SHOOT_NO_AIM` does not require Drive but retains geometric alignment gating.
- Neutral shoot and purge zones use a 1.25-motor-rotation hood lock.
- Manual presets are 2500 RPM / 0.0 rotations and 2900 RPM / 0.75 rotations.
- Autonomous Juicer transition occurs at the inclusive 1.5-second boundary.
- Hub Shift data is logged and exposed but never gates shooting.
- Register `Intake`, `Shoot`, `ShootNoAim`, and `Drive` before building the PathPlanner chooser.
- Do not modify mechanism IO configuration, motor tuning, PathPlanner path files, or Vision filtering.
- Use `Logger.recordOutput()`, not DogLog.
- Use `apply_patch` for hand edits and explicit path staging for commits.

## File structure

- Create `src/main/java/frc/robot/subsystems/SuperstructureTargeting.java`: pure alliance, zone, aim-point, heading, and alignment rules.
- Replace `src/main/java/frc/robot/subsystems/Superstructure.java`: state ownership, command factories, target/setpoint flow, drive heading state, and logging.
- Create `src/main/java/frc/robot/util/HubShiftUtil.java`: official/shifted game timing and dashboard override API.
- Modify `src/main/java/frc/robot/commands/DriveCommands.java`: add a heading-observer overload for `joystickDriveAtAngle`.
- Modify `src/main/java/frc/robot/RobotContainer.java`: construct Superstructure, register named commands, install defaults, and bind both controllers.
- Modify `src/main/java/frc/robot/Robot.java`: reset Hub Shift timing in autonomous and teleop initialization.
- Create `src/test/java/frc/robot/subsystems/SuperstructureTestHarness.java`: shared real-subsystem/no-hardware fixture with mutable Shooter inputs and recording Vision.
- Create focused tests for targeting, state commands, shooting, autonomous timing, bindings, Hub
  Shift, logging, coordinated physics simulation, and RobotContainer integration.

---

### Task 1: Add pure field-targeting rules

**Files:**
- Create: `src/main/java/frc/robot/subsystems/SuperstructureTargeting.java`
- Create: `src/test/java/frc/robot/subsystems/SuperstructureTargetingTest.java`

**Interfaces:**
- Consumes: `FieldConstants.FieldZones`, `FieldConstants.AimPoints`, `FieldConstants.Hub`, `DriverStation.Alliance`, `Pose2d`.
- Produces:
  - `static Translation2d resolveAimTarget(boolean allianceConfirmed, FieldZones zone, Alliance alliance)`
  - `static boolean isShootAllowed(boolean allianceConfirmed, FieldZones zone)`
  - `static boolean isPurgeZone(boolean allianceConfirmed, FieldZones zone)`
  - `static boolean isNeutralShootOrPurgeZone(boolean allianceConfirmed, FieldZones zone)`
  - `static Rotation2d geometricTargetHeading(Pose2d pose, Translation2d target)`
  - `static boolean isAligned(Pose2d pose, Translation2d target, double toleranceDegrees)`

- [ ] **Step 1: Write the failing targeting tests**

Create tests that exhaustively classify every zone, verify direct left/right mappings for both
alliances, verify hub fallback, and exercise angle wrapping:

```java
package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;
import java.util.EnumSet;
import org.junit.jupiter.api.Test;

class SuperstructureTargetingTest {
  private static final EnumSet<FieldZones> ALLOWED =
      EnumSet.of(
          FieldZones.ALLIANCE_LEFT,
          FieldZones.ALLIANCE_RIGHT,
          FieldZones.ALLIANCE_LEFT_TRENCH,
          FieldZones.ALLIANCE_RIGHT_TRENCH,
          FieldZones.NEUTRAL_LEFT_SHOOT,
          FieldZones.NEUTRAL_RIGHT_SHOOT,
          FieldZones.NEUTRAL_LEFT_PURGE,
          FieldZones.NEUTRAL_RIGHT_PURGE);

  @Test
  void shootAllowedSetIsExactAndRequiresConfirmedAlliance() {
    for (FieldZones zone : FieldZones.values()) {
      assertEquals(ALLOWED.contains(zone), SuperstructureTargeting.isShootAllowed(true, zone), zone.name());
      assertFalse(SuperstructureTargeting.isShootAllowed(false, zone), zone.name());
    }
    assertFalse(SuperstructureTargeting.isShootAllowed(true, null));
  }

  @Test
  void neutralAimPointsKeepAllianceRelativeLeftAndRight() {
    assertEquals(
        FieldConstants.AimPoints.BLUE_LEFT_SHOOT_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_LEFT_SHOOT, Alliance.Blue));
    assertEquals(
        FieldConstants.AimPoints.BLUE_RIGHT_PURGE_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_RIGHT_PURGE, Alliance.Blue));
    assertEquals(
        FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_LEFT_SHOOT, Alliance.Red));
    assertEquals(
        FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_RIGHT_PURGE, Alliance.Red));
  }

  @Test
  void missingAllianceConfirmationAlwaysFallsBackToBlueHub() {
    assertEquals(
        FieldConstants.Hub.BLUE_CENTER_POSE,
        SuperstructureTargeting.resolveAimTarget(false, null, Alliance.Blue));
    assertEquals(
        FieldConstants.Hub.BLUE_CENTER_POSE,
        SuperstructureTargeting.resolveAimTarget(false, null, Alliance.Red));
  }

  @Test
  void confirmedAllianceWithUnknownZoneFallsBackToItsHub() {
    assertEquals(
        FieldConstants.Hub.BLUE_CENTER_POSE,
        SuperstructureTargeting.resolveAimTarget(true, null, Alliance.Blue));
    assertEquals(
        FieldConstants.Hub.RED_CENTER_POSE,
        SuperstructureTargeting.resolveAimTarget(true, null, Alliance.Red));
  }

  @Test
  void alignmentUsesStrictFiveDegreeBoundaryAndWraps() {
    Translation2d target = new Translation2d(1.0, 0.0);
    assertTrue(
        SuperstructureTargeting.isAligned(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(4.999)), target, 5.0));
    assertFalse(
        SuperstructureTargeting.isAligned(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(5.0)), target, 5.0));
    assertTrue(
        SuperstructureTargeting.isAligned(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(359.0)), target, 5.0));
  }
}
```

- [ ] **Step 2: Run the targeting test and verify RED**

Run:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.SuperstructureTargetingTest" --console=plain --rerun-tasks --no-daemon
```

Expected: compilation fails because `SuperstructureTargeting` does not exist.

- [ ] **Step 3: Implement the pure targeting helper**

Create a package-private final class with no mutable state:

```java
final class SuperstructureTargeting {
  private SuperstructureTargeting() {}

  static Translation2d resolveAimTarget(
      boolean allianceConfirmed, FieldZones zone, Alliance alliance) {
    if (!allianceConfirmed) {
      return FieldConstants.Hub.BLUE_CENTER_POSE;
    }
    if (zone == null) {
      return alliance == Alliance.Red
          ? FieldConstants.Hub.RED_CENTER_POSE
          : FieldConstants.Hub.BLUE_CENTER_POSE;
    }
    boolean red = alliance == Alliance.Red;
    return switch (zone) {
      case NEUTRAL_LEFT_SHOOT ->
          red
              ? FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT
              : FieldConstants.AimPoints.BLUE_LEFT_SHOOT_POINT;
      case NEUTRAL_RIGHT_SHOOT ->
          red
              ? FieldConstants.AimPoints.RED_RIGHT_SHOOT_POINT
              : FieldConstants.AimPoints.BLUE_RIGHT_SHOOT_POINT;
      case NEUTRAL_LEFT_PURGE ->
          red
              ? FieldConstants.AimPoints.RED_LEFT_PURGE_POINT
              : FieldConstants.AimPoints.BLUE_LEFT_PURGE_POINT;
      case NEUTRAL_RIGHT_PURGE ->
          red
              ? FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT
              : FieldConstants.AimPoints.BLUE_RIGHT_PURGE_POINT;
      default ->
          red ? FieldConstants.Hub.RED_CENTER_POSE : FieldConstants.Hub.BLUE_CENTER_POSE;
    };
  }

  static boolean isShootAllowed(boolean allianceConfirmed, FieldZones zone) {
    if (!allianceConfirmed || zone == null) return false;
    return switch (zone) {
      case ALLIANCE_LEFT,
          ALLIANCE_RIGHT,
          ALLIANCE_LEFT_TRENCH,
          ALLIANCE_RIGHT_TRENCH,
          NEUTRAL_LEFT_SHOOT,
          NEUTRAL_RIGHT_SHOOT,
          NEUTRAL_LEFT_PURGE,
          NEUTRAL_RIGHT_PURGE -> true;
      default -> false;
    };
  }

  static boolean isPurgeZone(boolean allianceConfirmed, FieldZones zone) {
    return allianceConfirmed
        && (zone == FieldZones.NEUTRAL_LEFT_PURGE
            || zone == FieldZones.NEUTRAL_RIGHT_PURGE);
  }

  static boolean isNeutralShootOrPurgeZone(boolean allianceConfirmed, FieldZones zone) {
    return allianceConfirmed
        && (zone == FieldZones.NEUTRAL_LEFT_SHOOT
            || zone == FieldZones.NEUTRAL_RIGHT_SHOOT
            || zone == FieldZones.NEUTRAL_LEFT_PURGE
            || zone == FieldZones.NEUTRAL_RIGHT_PURGE);
  }

  static Rotation2d geometricTargetHeading(Pose2d pose, Translation2d target) {
    return new Rotation2d(
        Math.atan2(target.getY() - pose.getY(), target.getX() - pose.getX()));
  }

  static boolean isAligned(Pose2d pose, Translation2d target, double toleranceDegrees) {
    double error =
        Math.IEEEremainder(
            pose.getRotation().getRadians()
                - geometricTargetHeading(pose, target).getRadians(),
            2.0 * Math.PI);
    return Math.abs(Math.toDegrees(error)) < toleranceDegrees;
  }
}
```

- [ ] **Step 4: Run focused targeting tests and verify GREEN**

Run the Step 2 command. Expected: all `SuperstructureTargetingTest` tests pass.

- [ ] **Step 5: Commit the targeting unit**

```powershell
git add -- src/main/java/frc/robot/subsystems/SuperstructureTargeting.java src/test/java/frc/robot/subsystems/SuperstructureTargetingTest.java
git diff --cached --check
git commit -m "feat(superstructure): add field targeting rules"
```

---

### Task 2: Port deterministic Hub Shift timing

**Files:**
- Create: `src/main/java/frc/robot/util/HubShiftUtil.java`
- Create: `src/test/java/frc/robot/util/HubShiftUtilTest.java`

**Interfaces:**
- Consumes: Driver Station alliance, enabled/autonomous/FMS state, game-specific message, match time, and `Timer`.
- Produces:
  - `enum ShiftEnum` with `TRANSITION`, `SHIFT1`, `SHIFT2`, `SHIFT3`, `SHIFT4`, `ENDGAME`, `AUTO`, `DISABLED`
  - `record ShiftInfo(ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active)`
  - `static void initialize()`
  - `static Alliance getFirstActiveAlliance()`
  - `static ShiftInfo getOfficialShiftInfo()`
  - `static ShiftInfo getShiftedShiftInfo()`
  - `static void setAllianceWinOverride(Supplier<Optional<Boolean>> override)`

- [ ] **Step 1: Write deterministic Hub Shift tests**

Copy the fixed-timer fixture from the pinned reference
`src/test/java/frc/robot/util/HubShiftUtilTest.java` and extend it with exact shifted boundaries:

```java
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
  DriverStationSim.setMatchTime(100.0); // 40.0 elapsed in teleop
  DriverStationSim.notifyNewData();
  fixedTimer.setTime(20.0);

  ShiftInfo info = HubShiftUtil.getOfficialShiftInfo();

  assertEquals(ShiftEnum.SHIFT2, info.currentShift());
  assertEquals(5.0, info.elapsedTime(), 1e-9);
  assertEquals(20.0, info.remainingTime(), 1e-9);
}
```

Retain the reference test's official checkpoints at 5, 10, 35, 60, 85, and 110 seconds and its
reflection-based restoration of the static Timer and timer offset. Reset Driver Station data,
override supplier, Timer, and timer offset in `@AfterEach`.

- [ ] **Step 2: Run the Hub Shift tests and verify RED**

```powershell
.\gradlew.bat test --tests "frc.robot.util.HubShiftUtilTest" --console=plain --rerun-tasks --no-daemon
```

Expected: compilation fails because `HubShiftUtil` does not exist.

- [ ] **Step 3: Port the reference utility with explicit setter**

Port the pinned reference's production file exactly, remove Lombok, and replace the generated setter
with:

```java
private static Supplier<Optional<Boolean>> allianceWinOverride = Optional::empty;

public static void setAllianceWinOverride(
    Supplier<Optional<Boolean>> allianceWinOverride) {
  HubShiftUtil.allianceWinOverride = Objects.requireNonNull(allianceWinOverride);
}
```

Preserve these timing constants exactly:

```java
private static final double[] SHIFT_START_TIMES = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
private static final double[] SHIFT_END_TIMES = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};
private static final double APPROACHING_ACTIVE_FUDGE_SECONDS = -2.0;
private static final double ENDING_ACTIVE_FUDGE_SECONDS = -0.5;
private static final boolean[] STARTING_ACTIVE = {true, true, false, true, false, true};
private static final boolean[] STARTING_INACTIVE = {true, false, true, false, true, true};
```

Keep `getShiftInfo` private, allocate no lists in the loop, and preserve FMS resynchronization when
the timer differs from field teleop time by at least 3.0 seconds.

- [ ] **Step 4: Run Hub Shift tests and verify GREEN**

Run the Step 2 command. Expected: all official, shifted, auto, disabled, alliance, override, and FMS
resynchronization cases pass.

- [ ] **Step 5: Commit Hub Shift as an independent unit**

```powershell
git add -- src/main/java/frc/robot/util/HubShiftUtil.java src/test/java/frc/robot/util/HubShiftUtilTest.java
git diff --cached --check
git commit -m "feat(superstructure): add hub shift timing"
```

---

### Task 3: Build the Superstructure fixture and safe core states

**Files:**
- Replace: `src/main/java/frc/robot/subsystems/Superstructure.java`
- Create: `src/test/java/frc/robot/subsystems/SuperstructureTestHarness.java`
- Create: `src/test/java/frc/robot/subsystems/SuperstructureStateCommandTest.java`

**Interfaces:**
- Consumes: `Drive`, `Intake`, `Hopper`, `Shooter`, `Vision`, and `Supplier<Timer>`.
- Produces:
  - `public Superstructure(Drive drive, Intake intake, Hopper hopper, Shooter shooter, Vision vision)`
  - package-private constructor with final `Supplier<Timer> timerFactory` argument
  - `Superstate` and `ShootMode` enums from the approved spec
  - `Superstate getCurrentState()`
  - `ShootMode getShootMode()`
  - `Command setStateCmd(Superstate desiredState)`
  - `Command intakeOverrideCmd(IntakeState state)`
  - `Command hopperOverrideCmd(HopperState state)`
  - `Command shooterOverrideCmd(ShooterState state)`
  - test-fixture methods `setAlliance(AllianceStationID)`, `setPose(Pose2d)`,
    `makeShooterReady()`, and `poseInside(FieldZones)`

- [ ] **Step 1: Create the shared no-hardware test fixture**

The fixture initializes HAL once, uses a real no-op Drive and real mechanism state machines, and
records Vision aiming:

```java
public final class SuperstructureTestHarness implements AutoCloseable {
  public final MutableShooterIO shooterIO = new MutableShooterIO();
  public final Intake intake = new Intake(new IntakeIO.NoOp());
  public final Hopper hopper = new Hopper(new HopperIO.NoOp());
  public final Shooter shooter = new Shooter(shooterIO);
  public final RecordingDrive drive = new RecordingDrive();
  public final RecordingVision vision = new RecordingVision(drive);
  public final FixedTimer fixedTimer = new FixedTimer();
  public final Superstructure superstructure =
      new Superstructure(drive, intake, hopper, shooter, vision, () -> fixedTimer);

  public SuperstructureTestHarness() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  public void run(Command command) {
    command.schedule();
    runCycles(2);
  }

  public void runCycles(int count) {
    for (int cycle = 0; cycle < count; cycle++) {
      CommandScheduler.getInstance().run();
    }
  }

  public void setAlliance(AllianceStationID station) {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  public void setPose(Pose2d pose) {
    drive.setPoseAndSimulationTruth(pose);
    superstructure.periodic();
  }

  public void makeShooterReady() {
    shooterIO.flywheelRpm = shooter.getTargetRpm();
    shooterIO.hoodRotations = shooter.getTargetHoodRotations();
    shooter.periodic();
  }

  public Pose2d poseInside(FieldZones desiredZone) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    for (double x = 0.05; x < FieldConstants.FIELD_LENGTH; x += 0.05) {
      for (double y = 0.05; y < FieldConstants.FIELD_WIDTH; y += 0.05) {
        Pose2d candidate = new Pose2d(x, y, Rotation2d.kZero);
        if (FieldZones.fromPose(candidate, alliance) == desiredZone) {
          return candidate;
        }
      }
    }
    throw new IllegalArgumentException("No test pose found for " + desiredZone.name());
  }

  @Override
  public void close() {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    scheduler.unregisterSubsystem(superstructure, drive, intake, hopper, shooter, vision);
  }

  public static final class RecordingDrive extends Drive {
    private ChassisSpeeds lastRequestedSpeeds = new ChassisSpeeds();

    RecordingDrive() {
      super(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds) {
      lastRequestedSpeeds =
          new ChassisSpeeds(
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond,
              speeds.omegaRadiansPerSecond);
      super.runVelocity(speeds);
    }

    public ChassisSpeeds lastRequestedSpeeds() {
      return lastRequestedSpeeds;
    }
  }

  public static final class RecordingVision extends Vision {
    public boolean aiming;
    public int reseedCalls;

    RecordingVision(Drive drive) {
      super(
          VisionDriveBindings.fromDrive(drive),
          VisionRuntimeConfig.fromSystemProperties(),
          () -> {},
          new VisionIO.NoOp("superstructure-test"));
    }

    @Override
    public void setAiming(boolean aiming) {
      this.aiming = aiming;
    }

    @Override
    public boolean forceReseedFromVision() {
      reseedCalls++;
      return true;
    }

    @Override
    public void periodic() {
      // Avoid mutating Vision startup-strategy globals in Superstructure-only tests.
    }
  }

  public static final class MutableShooterIO implements ShooterIO {
    public double flywheelRpm;
    public double hoodRotations;
    public double kickerDutyCycle;
    public double kickerVoltage;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      inputs.flywheelLeadVelocityRpm = flywheelRpm;
      inputs.hoodPositionRotations = hoodRotations;
    }

    @Override
    public void setKickerDutyCycle(double output) {
      kickerDutyCycle = output;
    }

    @Override
    public void setKickerVoltage(Voltage voltage) {
      kickerVoltage = voltage.in(Volts);
    }
  }

  public static final class FixedTimer extends Timer {
    private double time;

    public void setTime(double time) {
      this.time = time;
    }

    @Override
    public double get() {
      return time;
    }

    @Override
    public void restart() {
      time = 0.0;
    }
  }
}
```

Each test class initializes HAL once. Its `@AfterEach` closes the harness, clears the active button
event loop when the class installs triggers, resets Driver Station simulation data, and calls
`notifyNewData()`. This prevents singleton scheduler registrations and HID trigger callbacks from
leaking across tests:

```java
@AfterEach
void tearDown() {
  harness.close();
  CommandScheduler.getInstance().getActiveButtonLoop().clear();
  DriverStationSim.resetData();
  DriverStationSim.notifyNewData();
}
```

- [ ] **Step 2: Write failing safe-start and state mapping tests**

```java
@Test
void defaultsAndLoggedStateUseSafeStartingConfiguration() {
  assertEquals(Superstate.DRIVE_STARTING_CONFIG, harness.superstructure.getCurrentState());
  assertEquals(IntakeState.HOME, harness.intake.getDesiredState());
  assertEquals(HopperState.STOP, harness.hopper.getDesiredState());
  assertEquals(ShooterState.PREPFUEL, harness.shooter.getDesiredState());
}

@Test
void coreStatesRequestExactMechanismStates() {
  assertState(Superstate.DRIVE, IntakeState.DEPLOYED, HopperState.STOP, ShooterState.PREPFUEL);
  assertState(Superstate.INTAKE, IntakeState.INTAKE, HopperState.STOP, ShooterState.PREPFUEL);
  assertState(
      Superstate.OUTTAKE,
      IntakeState.OUTTAKE,
      HopperState.INDEX_TO_INTAKE,
      ShooterState.PREPFUEL);
  assertState(
      Superstate.DRIVE_STARTING_CONFIG,
      IntakeState.HOME,
      HopperState.STOP,
      ShooterState.PREPFUEL);
}

private void assertState(
    Superstate state,
    IntakeState intakeState,
    HopperState hopperState,
    ShooterState shooterState) {
  harness.run(harness.superstructure.setStateCmd(state));
  assertEquals(state, harness.superstructure.getCurrentState());
  assertEquals(intakeState, harness.intake.getDesiredState());
  assertEquals(hopperState, harness.hopper.getDesiredState());
  assertEquals(shooterState, harness.shooter.getDesiredState());
}
```

Also assert null constructor dependencies and null requested Superstructure/mechanism states throw
`NullPointerException`. Task 4 adds the mode factories and their null-mode assertions.

- [ ] **Step 3: Run the core state tests and verify RED**

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.SuperstructureStateCommandTest" --console=plain --rerun-tasks --no-daemon
```

Expected: constructor and command APIs are missing from the placeholder class.

- [ ] **Step 4: Implement constructor, defaults, enums, and core commands**

Use continuous default commands and an exact state switch:

```java
private void configureSafeDefaults() {
  intake.setDesiredState(IntakeState.HOME);
  hopper.setDesiredState(HopperState.STOP);
  shooter.setDesiredState(ShooterState.PREPFUEL);

  intake.setDefaultCommand(
      Commands.idle(intake).withName("Intake.Default(HOLD_LAST_REQUEST)"));
  hopper.setDefaultCommand(
      Commands.run(() -> hopper.setDesiredState(HopperState.STOP), hopper)
          .withName("Hopper.Default(STOP)"));
  shooter.setDefaultCommand(
      Commands.run(() -> shooter.setDesiredState(ShooterState.PREPFUEL), shooter)
          .withName("Shooter.Default(PREPFUEL)"));
}

public Command setStateCmd(Superstate desiredState) {
  Objects.requireNonNull(desiredState);
  return switch (desiredState) {
    case DRIVE_STARTING_CONFIG ->
        mechanismStateCommand(
            desiredState, IntakeState.HOME, HopperState.STOP, ShooterState.PREPFUEL);
    case DRIVE ->
        mechanismStateCommand(
            desiredState, IntakeState.DEPLOYED, HopperState.STOP, ShooterState.PREPFUEL);
    case INTAKE ->
        mechanismStateCommand(
            desiredState, IntakeState.INTAKE, HopperState.STOP, ShooterState.PREPFUEL);
    case OUTTAKE ->
        mechanismStateCommand(
            desiredState,
            IntakeState.OUTTAKE,
            HopperState.INDEX_TO_INTAKE,
            ShooterState.PREPFUEL);
    case SHOOT_WITH_AIM, SHOOT_NO_AIM, MANUAL_SHOOT ->
        shootingPreparationCommand(desiredState);
    case PURGE -> purgePreparationCommand();
  };
}

private Command mechanismStateCommand(
    Superstate superstate,
    IntakeState intakeState,
    HopperState hopperState,
    ShooterState shooterState) {
  return Commands.run(
          () -> {
            intake.setDesiredState(intakeState);
            hopper.setDesiredState(hopperState);
            shooter.setDesiredState(shooterState);
            currentState = superstate;
          },
          this,
          intake,
          hopper,
          shooter)
      .withName("Superstate(" + superstate.name() + ")");
}

private Command shootingPreparationCommand(Superstate superstate) {
  return Commands.run(
          () -> {
            hopper.setDesiredState(HopperState.STOP);
            shooter.setDesiredState(ShooterState.SHOOT);
            currentState = superstate;
          },
          this,
          hopper,
          shooter)
      .withName("Superstate(" + superstate.name() + ":PREPARING)");
}

private Command purgePreparationCommand() {
  return Commands.run(
          () -> {
            intake.setDesiredState(IntakeState.OUTTAKE);
            hopper.setDesiredState(HopperState.STOP);
            shooter.setDesiredState(ShooterState.SHOOT);
            currentState = Superstate.PURGE;
          },
          this,
          intake,
          hopper,
          shooter)
      .withName("Superstate(PURGE:PREPARING)");
}
```

The constructor seeds Intake `HOME`; its requirement-owning idle default deliberately preserves the
last explicit Intake request. This is required so a shoot command that does not own Intake leaves it
unchanged instead of stowing it when the preceding `DRIVE` command is interrupted. Hopper and
Shooter defaults continuously restore `STOP` and `PREPFUEL` whenever no coordinated command owns
them.

The four shooting cases deliberately compile to a safe, concrete preparation phase in this commit:
they leave Intake unchanged (or request outtake for purge), spin the Shooter, and keep Hopper
stopped. Task 4 replaces those switch arms with the complete readiness/feed/setpoint commands, and
Task 5 composes Drive ownership plus autonomous timing. This is an executable intermediate state,
not a stub. Implement every override command with `Commands.run` and only its mechanism requirement,
so an override remains authoritative for exactly as long as it is scheduled despite the continuous
mechanism defaults.

- [ ] **Step 5: Run core tests and the existing mechanism suites**

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.SuperstructureStateCommandTest" --tests "frc.robot.subsystems.intake.IntakeTest" --tests "frc.robot.subsystems.hopper.HopperTest" --tests "frc.robot.subsystems.shooter.ShooterTest" --console=plain --rerun-tasks --no-daemon
```

Expected: all selected tests pass.

- [ ] **Step 6: Commit the safe coordinator core**

```powershell
git add -- src/main/java/frc/robot/subsystems/Superstructure.java src/test/java/frc/robot/subsystems/SuperstructureTestHarness.java src/test/java/frc/robot/subsystems/SuperstructureStateCommandTest.java
git diff --cached --check
git commit -m "feat(superstructure): coordinate mechanism states"
```

---

### Task 4: Add shooting, targeting, setpoints, and feed gating

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/Superstructure.java`
- Create: `src/test/java/frc/robot/subsystems/SuperstructureShootingTest.java`
- Create: `src/test/java/frc/robot/subsystems/SuperstructureAimTargetTest.java`

**Interfaces:**
- Consumes Task 1 targeting methods and Task 3 state/fixture APIs.
- Produces:
  - `boolean isAlignedToTarget()`
  - `boolean isShootAllowed()`
  - `boolean shouldUsePurgeDuringShoot()`
  - `boolean isSelectedShootAllowed()`
  - `Command selectedShootModeCmd()`
  - `Command purgeShootCmd()`
  - `Command setShootModeCmd(ShootMode mode)`
  - `Command toggleShootModeCmd(ShootMode manualMode)`
  - `Translation2d getCurrentAimTarget()` with package-private visibility for tests

- [ ] **Step 1: Write failing shooting and aiming tests**

Cover shooter state plus alignment, manual bypass, mode toggles, vision aiming, target distance, and
the neutral hood limit. Assert both `setShootModeCmd(null)` and `toggleShootModeCmd(null)` throw
`NullPointerException` when their factories are called:

```java
@Test
void automaticFeedRequiresShooterShootStateAndAlignment() {
  harness.setAlliance(AllianceStationID.Blue1);
  harness.setPose(new Pose2d(1.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.kZero));
  Command command = harness.superstructure.setStateCmd(Superstate.SHOOT_NO_AIM);
  harness.run(command);
  assertEquals(HopperState.STOP, harness.hopper.getDesiredState());

  harness.makeShooterReady();
  CommandScheduler.getInstance().run();
  assertEquals(HopperState.INDEX_TO_SHOOTER, harness.hopper.getDesiredState());

  harness.setPose(
      new Pose2d(1.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.fromDegrees(5.0)));
  harness.superstructure.periodic();
  CommandScheduler.getInstance().run();
  assertEquals(HopperState.STOP, harness.hopper.getDesiredState());
}

@Test
void manualModesUseApprovedPresetsAndIgnoreAlignment() {
  harness.setPose(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(90.0)));
  harness.run(harness.superstructure.setShootModeCmd(ShootMode.MANUAL_BUMPER_UP));
  harness.run(harness.superstructure.selectedShootModeCmd());
  harness.superstructure.periodic();
  assertEquals(2500.0, harness.shooter.getTargetRpm());
  assertEquals(0.0, harness.shooter.getTargetHoodRotations());

  harness.run(harness.superstructure.setShootModeCmd(ShootMode.MANUAL_TRENCH));
  harness.run(harness.superstructure.selectedShootModeCmd());
  harness.superstructure.periodic();
  assertEquals(2900.0, harness.shooter.getTargetRpm());
  assertEquals(0.75, harness.shooter.getTargetHoodRotations());
}

@Test
void everyNeutralShootAndPurgeZoneCapsHoodWithoutChangingInterpolatedRpm() {
  harness.setAlliance(AllianceStationID.Blue1);
  for (FieldZones zone :
      List.of(
          FieldZones.NEUTRAL_LEFT_SHOOT,
          FieldZones.NEUTRAL_RIGHT_SHOOT,
          FieldZones.NEUTRAL_LEFT_PURGE,
          FieldZones.NEUTRAL_RIGHT_PURGE)) {
    harness.setPose(harness.poseInside(zone));
    harness.superstructure.periodic();
    assertTrue(harness.shooter.getTargetRpm() > 0.0, zone.name());
    assertEquals(1.25, harness.shooter.getTargetHoodRotations(), zone.name());
  }
}
```

Port the reference geometry cases named
`redAllianceFacingGeometricTargetIsAligned`,
`geometricAlignmentIsSameForBlueAndRed`,
`rawGeometricHeadingDoesNotAddRedOperatorPerspectivePi`,
`redRightAimPointsMirrorRedLeftAcrossHorizontalCenterline`,
`redAllianceNeutralZonesUseDirectLeftRightAimPoints`,
`blueAllianceNeutralZonesKeepStandardLeftRightAimPoints`, and
`defaultsToAllianceHubWhenZoneUnknownAndBlueWhenUnconfirmed`. Update their calls to
`SuperstructureTargeting` and retain their exact blue/red expected points. The replacement raw-heading
test must assert that both alliances use the direct `atan2` field heading; do not add the unused
CTRE operator-perspective `+pi` helper from the reference.

- [ ] **Step 2: Run focused tests and verify RED**

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.SuperstructureShootingTest" --tests "frc.robot.subsystems.SuperstructureAimTargetTest" --console=plain --rerun-tasks --no-daemon
```

Expected: shooting APIs and periodic target behavior are absent.

- [ ] **Step 3: Implement periodic target state**

Add constants and exact target flow:

```java
private static final double ALIGNMENT_TOLERANCE_DEGREES = 5.0;
private static final double NEUTRAL_ZONE_HOOD_ROTATIONS = 1.25;
private static final double MANUAL_BUMPER_UP_RPM = 2500.0;
private static final double MANUAL_BUMPER_UP_HOOD_ROTATIONS = 0.0;
private static final double MANUAL_TRENCH_RPM = 2900.0;
private static final double MANUAL_TRENCH_HOOD_ROTATIONS = 0.75;

private Alliance alliance = Alliance.Blue;
private boolean allianceConfirmed;
private FieldZones currentZone;
private Translation2d currentAimTarget = FieldConstants.Hub.BLUE_CENTER_POSE;
private double distanceToTargetMeters;
private boolean alignedToTarget;

private void updateTargeting(Pose2d pose) {
  Optional<Alliance> dsAlliance = DriverStation.getAlliance();
  if (dsAlliance.isPresent()) {
    alliance = dsAlliance.orElseThrow();
    allianceConfirmed = true;
    currentZone = FieldZones.fromPose(pose, alliance);
  } else {
    alliance = Alliance.Blue;
    allianceConfirmed = false;
    currentZone = null;
  }

  currentAimTarget =
      SuperstructureTargeting.resolveAimTarget(allianceConfirmed, currentZone, alliance);
  distanceToTargetMeters = pose.getTranslation().getDistance(currentAimTarget);
  alignedToTarget =
      SuperstructureTargeting.isAligned(
          pose, currentAimTarget, ALIGNMENT_TOLERANCE_DEGREES);

  if (currentState != Superstate.MANUAL_SHOOT) {
    shooter.setSetpointForDistance(distanceToTargetMeters);
    if (SuperstructureTargeting.isNeutralShootOrPurgeZone(
        allianceConfirmed, currentZone)) {
      shooter.setSetpoint(shooter.getTargetRpm(), NEUTRAL_ZONE_HOOD_ROTATIONS);
    }
  }
}
```

`periodic()` calls `updateTargeting(drive.getPose())`, updates Vision aiming only for
`SHOOT_WITH_AIM` and `SHOOT_NO_AIM`, and records outputs in Task 8.

- [ ] **Step 4: Implement shooting and feed commands**

```java
private void setHopperFeedWhenReady(boolean requireAlignment) {
  boolean shooterReady = shooter.getCurrentState() == ShooterState.SHOOT;
  boolean alignmentReady = !requireAlignment || alignedToTarget;
  hopper.setDesiredState(
      shooterReady && alignmentReady ? HopperState.INDEX_TO_SHOOTER : HopperState.STOP);
}

private Command createShootMechanismCommand(
    Superstate state, boolean requireAlignment) {
  return Commands.run(
      () -> {
        shooter.setDesiredState(ShooterState.SHOOT);
        setHopperFeedWhenReady(requireAlignment);
        currentState = state;
      },
      this,
      shooter,
      hopper);
}

public boolean isSelectedShootAllowed() {
  return shootMode != ShootMode.DEFAULT_SHOOT_WITH_AIM
      || SuperstructureTargeting.isShootAllowed(allianceConfirmed, currentZone);
}

public boolean isShootAllowed() {
  return SuperstructureTargeting.isShootAllowed(allianceConfirmed, currentZone);
}

public boolean shouldUsePurgeDuringShoot() {
  return shootMode == ShootMode.DEFAULT_SHOOT_WITH_AIM
      && SuperstructureTargeting.isPurgeZone(allianceConfirmed, currentZone);
}

public boolean isAlignedToTarget() {
  return alignedToTarget;
}

Translation2d getCurrentAimTarget() {
  return currentAimTarget;
}
```

Replace Task 3's preparation switch arms with these exact final mechanism routes:

- `SHOOT_WITH_AIM`: return a requirement-free named `Commands.idle()` when disallowed, route purge
  zones to the purge command, otherwise call `createShootMechanismCommand(..., true)`;
- `SHOOT_NO_AIM`: return requirement-free idle when disallowed, otherwise call
  `createShootMechanismCommand(..., true)` so geometric alignment remains required;
- `MANUAL_SHOOT`: set the approved selected preset every loop, command Shooter `SHOOT`, feed with
  `requireAlignment=false`, and combine with `Commands.run(drive::stopWithX, drive)`;
- `PURGE`: return requirement-free idle outside purge zones; inside a purge zone require
  Intake/Shooter/Hopper, request Intake `OUTTAKE`, and use alignment-gated feed.

`selectedShootModeCmd()` switches on the current mode when the factory is invoked. Task 7 invokes it
inside `Commands.defer`, so right-trigger mode and zone routing are evaluated at schedule time.
Task 5 adds stationary Drive aiming to the aimed and purge routes without changing their mechanism
requirements.

Implement the complete factory/mode surface now (Task 5 only adds Drive composition):

```java
private Command shootWithAimCmd() {
  if (!isShootAllowed()) {
    return Commands.idle().withName("Superstate(SHOOT_WITH_AIM:DISALLOWED)");
  }
  if (SuperstructureTargeting.isPurgeZone(allianceConfirmed, currentZone)) {
    return createPurgeMechanismCommand()
        .withName("Superstate(SHOOT_WITH_AIM->PURGE)");
  }
  return createShootMechanismCommand(Superstate.SHOOT_WITH_AIM, true)
      .withName("Superstate(SHOOT_WITH_AIM)");
}

private Command shootNoAimCmd() {
  if (!isShootAllowed()) {
    return Commands.idle().withName("Superstate(SHOOT_NO_AIM:DISALLOWED)");
  }
  return createShootMechanismCommand(Superstate.SHOOT_NO_AIM, true)
      .withName("Superstate(SHOOT_NO_AIM)");
}

private Command createManualShootStateCommand(double rpm, double hoodRotations) {
  return Commands.run(
          () -> {
            shooter.setSetpoint(rpm, hoodRotations);
            shooter.setDesiredState(ShooterState.SHOOT);
            setHopperFeedWhenReady(false);
            currentState = Superstate.MANUAL_SHOOT;
          },
          this,
          shooter,
          hopper)
      .alongWith(Commands.run(drive::stopWithX, drive));
}

private Command manualShootCmd() {
  return createManualShootStateCommand(
          shooter.getTargetRpm(), shooter.getTargetHoodRotations())
      .withName("Superstate(MANUAL_SHOOT)");
}

private Command createPurgeMechanismCommand() {
  return Commands.run(
      () -> {
        intake.setDesiredState(IntakeState.OUTTAKE);
        shooter.setDesiredState(ShooterState.SHOOT);
        setHopperFeedWhenReady(true);
        currentState = Superstate.PURGE;
      },
      this,
      intake,
      shooter,
      hopper);
}

private Command purgeCmd() {
  if (!SuperstructureTargeting.isPurgeZone(allianceConfirmed, currentZone)) {
    return Commands.idle().withName("Superstate(PURGE:DISALLOWED)");
  }
  return createPurgeMechanismCommand().withName("Superstate(PURGE)");
}

public Command selectedShootModeCmd() {
  return switch (shootMode) {
    case DEFAULT_SHOOT_WITH_AIM -> shootWithAimCmd();
    case MANUAL_BUMPER_UP ->
        createManualShootStateCommand(
                MANUAL_BUMPER_UP_RPM, MANUAL_BUMPER_UP_HOOD_ROTATIONS)
            .withName("Superstate(SHOOT->MANUAL_BUMPER_UP)");
    case MANUAL_TRENCH ->
        createManualShootStateCommand(
                MANUAL_TRENCH_RPM, MANUAL_TRENCH_HOOD_ROTATIONS)
            .withName("Superstate(SHOOT->MANUAL_TRENCH)");
  };
}

public Command purgeShootCmd() {
  return purgeCmd().withName("Superstate(SHOOT->PURGE)");
}

public Command setShootModeCmd(ShootMode mode) {
  Objects.requireNonNull(mode);
  return Commands.runOnce(() -> shootMode = mode)
      .withName("SetShootMode(" + mode.name() + ")");
}

public Command toggleShootModeCmd(ShootMode manualMode) {
  Objects.requireNonNull(manualMode);
  return Commands.runOnce(
          () ->
              shootMode =
                  shootMode == manualMode
                      ? ShootMode.DEFAULT_SHOOT_WITH_AIM
                      : manualMode)
      .withName("ToggleShootMode(" + manualMode.name() + ")");
}
```

Expose the simple state predicates/getter from the same cached fields. At the end of `periodic()`,
call `vision.setAiming(currentState == Superstate.SHOOT_WITH_AIM || currentState ==
Superstate.SHOOT_NO_AIM)`. Tests must observe `true` in exactly those two states and `false` in
`DRIVE`, `MANUAL_SHOOT`, and `PURGE`.

- [ ] **Step 5: Run shooting, targeting, and existing subsystem tests**

Run the Step 2 command, followed by:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.shooter.*" --tests "frc.robot.subsystems.hopper.*" --tests "frc.robot.FieldConstantsTest" --console=plain --rerun-tasks --no-daemon
```

Expected: all selected tests pass.

- [ ] **Step 6: Commit shooting coordination**

```powershell
git add -- src/main/java/frc/robot/subsystems/Superstructure.java src/test/java/frc/robot/subsystems/SuperstructureShootingTest.java src/test/java/frc/robot/subsystems/SuperstructureAimTargetTest.java
git diff --cached --check
git commit -m "feat(superstructure): coordinate shooting states"
```

---

### Task 5: Add stationary Drive aiming and delayed autonomous shots

**Files:**
- Modify: `src/main/java/frc/robot/commands/DriveCommands.java`
- Modify: `src/main/java/frc/robot/subsystems/Superstructure.java`
- Modify: `src/test/java/frc/robot/commands/DriveCommandsTest.java`
- Create: `src/test/java/frc/robot/subsystems/SuperstructureCommandRequirementsTest.java`
- Create: `src/test/java/frc/robot/subsystems/SuperstructureAutonomousTest.java`
- Create: `src/test/java/frc/robot/subsystems/SuperstructureSimulationTest.java`

**Interfaces:**
- Consumes: `DriveCommands.joystickDriveAtAngle` and Task 4 aim target.
- Produces:
  - overload `joystickDriveAtAngle(Drive, DoubleSupplier, DoubleSupplier, Supplier<Rotation2d>, Consumer<Optional<Rotation2d>>)`
  - `Optional<Rotation2d> getCurrentHeading()` and setter
  - rotation-trigger timestamp getter and setter
  - `Optional<Rotation2d> getZoneLockedHeading()`
  - `Command shootWithJuicerDelayCmd()`
  - `Command shootNoAimWithJuicerDelayCmd()`
  - `Command forceReseedFromVisionCmd()`

- [ ] **Step 1: Write failing Drive heading-observer test**

Add a self-contained no-op Drive fixture to the existing pure `DriveCommandsTest`. Initialize HAL,
enable Driver Station simulation, and unregister the Drive in `finally` so the singleton scheduler
does not leak it:

```java
@Test
void joystickDriveAtAnglePublishesTheActiveTargetHeading() {
  AtomicReference<Optional<Rotation2d>> observed = new AtomicReference<>(Optional.empty());
  Rotation2d target = Rotation2d.fromDegrees(37.0);
  Drive drive =
      new Drive(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
  try {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Command command =
        DriveCommands.joystickDriveAtAngle(
            drive, () -> 0.0, () -> 0.0, () -> target, observed::set);
    command.schedule();
    CommandScheduler.getInstance().run();
    CommandScheduler.getInstance().run();
    assertEquals(Optional.of(target), observed.get());
  } finally {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterSubsystem(drive);
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }
}
```

Call `HAL.initialize(500, 0)` from the class's `@BeforeAll` method.

- [ ] **Step 2: Write command ownership and delayed-Juicer tests**

```java
@Test
void noAimDoesNotRequireDriveButAimedAndManualShotsDo() {
  harness.setAlliance(AllianceStationID.Blue1);
  harness.setPose(
      new Pose2d(1.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.kZero));
  assertFalse(
      harness.superstructure.setStateCmd(Superstate.SHOOT_NO_AIM)
          .getRequirements()
          .contains(harness.drive));
  assertFalse(
      harness.superstructure.setStateCmd(Superstate.SHOOT_NO_AIM)
          .getRequirements()
          .contains(harness.intake));
  assertTrue(
      harness.superstructure.setStateCmd(Superstate.SHOOT_WITH_AIM)
          .getRequirements()
          .contains(harness.drive));
  assertTrue(
      harness.superstructure.setStateCmd(Superstate.MANUAL_SHOOT)
          .getRequirements()
          .contains(harness.drive));
  assertFalse(
      harness.superstructure.setStateCmd(Superstate.MANUAL_SHOOT)
          .getRequirements()
          .contains(harness.intake));
  assertTrue(
      harness.superstructure.shootWithJuicerDelayCmd()
          .getRequirements()
          .contains(harness.drive));
  assertFalse(
      harness.superstructure.shootNoAimWithJuicerDelayCmd()
          .getRequirements()
          .contains(harness.drive));
}

@Test
void disallowedDefaultShotHasNoRequirementsAndChangesNoMechanisms() {
  harness.setAlliance(AllianceStationID.Blue1);
  harness.setPose(harness.poseInside(FieldZones.ALLIANCE_LEFT_BUMP));
  IntakeState intakeBefore = harness.intake.getDesiredState();
  HopperState hopperBefore = harness.hopper.getDesiredState();
  ShooterState shooterBefore = harness.shooter.getDesiredState();

  Command disallowed = harness.superstructure.selectedShootModeCmd();
  assertTrue(disallowed.getRequirements().isEmpty());
  harness.run(disallowed);

  assertEquals(intakeBefore, harness.intake.getDesiredState());
  assertEquals(hopperBefore, harness.hopper.getDesiredState());
  assertEquals(shooterBefore, harness.shooter.getDesiredState());
}

@Test
void regularAimedAndPurgeShotsOwnDriveAndRequestZeroTranslation() {
  harness.setAlliance(AllianceStationID.Blue1);
  harness.setPose(
      new Pose2d(
          1.0,
          FieldConstants.FIELD_WIDTH / 2.0,
          Rotation2d.fromDegrees(20.0)));
  Command aimed = harness.superstructure.setStateCmd(Superstate.SHOOT_WITH_AIM);
  assertTrue(aimed.getRequirements().contains(harness.drive));
  assertFalse(aimed.getRequirements().contains(harness.intake));
  harness.run(harness.superstructure.setStateCmd(Superstate.DRIVE));
  assertEquals(IntakeState.DEPLOYED, harness.intake.getDesiredState());
  harness.run(aimed);
  assertEquals(
      IntakeState.DEPLOYED,
      harness.intake.getDesiredState(),
      "regular shooting must preserve the prior Intake request");
  assertEquals(0.0, harness.drive.lastRequestedSpeeds().vxMetersPerSecond, 1e-9);
  assertEquals(0.0, harness.drive.lastRequestedSpeeds().vyMetersPerSecond, 1e-9);

  aimed.cancel();
  harness.setPose(harness.poseInside(FieldZones.NEUTRAL_LEFT_PURGE));
  Command purge = harness.superstructure.purgeShootCmd();
  assertTrue(purge.getRequirements().contains(harness.drive));
  assertTrue(purge.getRequirements().contains(harness.intake));
  assertTrue(
      harness.superstructure.setStateCmd(Superstate.PURGE)
          .getRequirements()
          .contains(harness.drive));
  harness.run(purge);
  assertEquals(0.0, harness.drive.lastRequestedSpeeds().vxMetersPerSecond, 1e-9);
  assertEquals(0.0, harness.drive.lastRequestedSpeeds().vyMetersPerSecond, 1e-9);
}

@Test
void autonomousShootChangesToJuicerAtInclusiveOnePointFiveSeconds() {
  Command command = harness.superstructure.shootNoAimWithJuicerDelayCmd();
  harness.run(command);
  assertEquals(IntakeState.DEPLOYED, harness.intake.getDesiredState());

  harness.fixedTimer.setTime(1.499);
  CommandScheduler.getInstance().run();
  assertEquals(IntakeState.DEPLOYED, harness.intake.getDesiredState());

  harness.fixedTimer.setTime(1.500);
  CommandScheduler.getInstance().run();
  assertEquals(IntakeState.JUICER, harness.intake.getDesiredState());
}

@Test
void headingStateZoneLockAndVisionReseedExposeTheRequiredContainerSeams() {
  harness.setAlliance(AllianceStationID.Blue1);
  harness.setPose(harness.poseInside(FieldZones.ALLIANCE_LEFT));
  Optional<Rotation2d> heading = Optional.of(Rotation2d.fromDegrees(17.0));
  harness.superstructure.setCurrentHeading(heading);
  harness.superstructure.setRotationLastTriggered(12.5);

  assertEquals(heading, harness.superstructure.getCurrentHeading());
  assertEquals(12.5, harness.superstructure.getRotationLastTriggered(), 1e-9);
  assertEquals(
      DriveCommands.getZoneLockedHeading(FieldZones.ALLIANCE_LEFT, Alliance.Blue),
      harness.superstructure.getZoneLockedHeading());

  Command reseed = harness.superstructure.forceReseedFromVisionCmd();
  assertTrue(reseed.getRequirements().contains(harness.vision));
  harness.run(reseed);
  assertEquals(1, harness.vision.reseedCalls);
}
```

Add a purge-zone case asserting the aimed autonomous command uses `OUTTAKE` for the entire run. Use
`harness.run()` (two scheduler cycles) before the first assertion because the command begins with a
`runOnce(...).andThen(...)` sequence.

Add a coordinated physics test using four `ModuleIOSim` instances plus `IntakeIOSim`, `HopperIOSim`,
and `ShooterIOSim`. Pause simulation timing, set the battery to 12 V, enable Blue Driver Station,
and start at an allowed alliance-zone pose with a 20-degree heading error. Schedule `INTAKE` long
enough for the simulated arm to reach `IntakeState.INTAKE`; then schedule the regular default aimed
shot and step 20 ms loops until ready. Assert all of the following:

```java
assertEquals(IntakeState.INTAKE, intake.getCurrentState());
assertEquals(ShooterState.SHOOT, shooter.getCurrentState());
assertEquals(HopperState.INDEX_TO_SHOOTER, hopper.getDesiredState());
assertTrue(superstructure.isAlignedToTarget());
assertTrue(
    drive.getPose().getTranslation().getDistance(startPose.getTranslation()) < 0.25);
```

This proves the aimed command rotates without translating while the existing full mechanism physics
models deploy, spin up, and feed together. Use a bounded loop (for example 700 cycles) that exits
early when all assertions' readiness conditions are true; never use wall-clock sleep. In `finally`,
cancel commands, unregister all five subsystems plus Superstructure, restore battery/timing/Driver
Station state, and clear the active button loop.

- [ ] **Step 3: Run focused tests and verify RED**

```powershell
.\gradlew.bat test --tests "frc.robot.commands.DriveCommandsTest" --tests "frc.robot.subsystems.SuperstructureCommandRequirementsTest" --tests "frc.robot.subsystems.SuperstructureAutonomousTest" --tests "frc.robot.subsystems.SuperstructureSimulationTest" --console=plain --rerun-tasks --no-daemon
```

Expected: overload and autonomous command factories are missing.

- [ ] **Step 4: Add the DriveCommands overload without breaking callers**

Keep the existing four-argument method and delegate:

```java
public static Command joystickDriveAtAngle(
    Drive drive,
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    Supplier<Rotation2d> rotationSupplier) {
  return joystickDriveAtAngle(
      drive, translationSup, strafeSup, rotationSupplier, ignored -> {});
}
```

In the new overload's execute lambda, evaluate the target once and publish it before PID:

```java
Rotation2d targetRotation = rotationSupplier.get();
headingObserver.accept(Optional.of(targetRotation));
double omega =
    angleController.calculate(
        drive.getRotation().getRadians(), targetRotation.getRadians());
```

- [ ] **Step 5: Compose stationary aim and autonomous commands**

```java
private Command stationaryAimCommand() {
  return DriveCommands.joystickDriveAtAngle(
      drive,
      () -> 0.0,
      () -> 0.0,
      () -> SuperstructureTargeting.geometricTargetHeading(drive.getPose(), currentAimTarget),
      this::setCurrentHeading);
}

private Optional<Rotation2d> currentHeading = Optional.empty();
private double rotationLastTriggered;

public Optional<Rotation2d> getCurrentHeading() {
  return currentHeading;
}

public void setCurrentHeading(Optional<Rotation2d> heading) {
  currentHeading = Objects.requireNonNull(heading);
}

public double getRotationLastTriggered() {
  return rotationLastTriggered;
}

public void setRotationLastTriggered(double timestampSeconds) {
  rotationLastTriggered = timestampSeconds;
}

public Optional<Rotation2d> getZoneLockedHeading() {
  if (!allianceConfirmed || currentZone == null) return Optional.empty();
  return DriveCommands.getZoneLockedHeading(currentZone, alliance);
}

public Command forceReseedFromVisionCmd() {
  return Commands.runOnce(vision::forceReseedFromVision, vision)
      .withName("Force Vision Reseed");
}

public Command shootNoAimWithJuicerDelayCmd() {
  Timer timer = Objects.requireNonNull(timerFactory.get());
  return Commands.runOnce(timer::restart)
      .andThen(
          Commands.run(
              () -> {
                currentState = Superstate.SHOOT_NO_AIM;
                intake.setDesiredState(
                    timer.hasElapsed(1.5) ? IntakeState.JUICER : IntakeState.DEPLOYED);
                shooter.setDesiredState(ShooterState.SHOOT);
                setHopperFeedWhenReady(true);
              },
              this,
              intake,
              shooter,
              hopper))
      .withName("Superstate(SHOOT_NO_AIM+JUICER)");
}

public Command shootWithJuicerDelayCmd() {
  Timer timer = Objects.requireNonNull(timerFactory.get());
  return Commands.runOnce(timer::restart)
      .andThen(
          Commands.run(
                  () -> {
                    boolean purge =
                        SuperstructureTargeting.isPurgeZone(
                            allianceConfirmed, currentZone);
                    currentState = purge ? Superstate.PURGE : Superstate.SHOOT_WITH_AIM;
                    intake.setDesiredState(
                        purge
                            ? IntakeState.OUTTAKE
                            : timer.hasElapsed(1.5)
                                ? IntakeState.JUICER
                                : IntakeState.DEPLOYED);
                    shooter.setDesiredState(ShooterState.SHOOT);
                    setHopperFeedWhenReady(true);
                  },
                  this,
                  intake,
                  shooter,
                  hopper)
              .alongWith(stationaryAimCommand()))
      .withName("Superstate(SHOOT_WITH_AIM+JUICER)");
}
```

Replace Task 4's existing `shootWithAimCmd()` and `purgeCmd()` definitions with the versions below;
do not add duplicate methods. This retrofits every regular aimed route explicitly:

```java
private Command shootWithAimCmd() {
  if (!isShootAllowed()) {
    return Commands.idle().withName("Superstate(SHOOT_WITH_AIM:DISALLOWED)");
  }
  if (SuperstructureTargeting.isPurgeZone(allianceConfirmed, currentZone)) {
    return purgeCmd();
  }
  return createShootMechanismCommand(Superstate.SHOOT_WITH_AIM, true)
      .alongWith(stationaryAimCommand())
      .withName("Superstate(SHOOT_WITH_AIM)");
}

private Command purgeCmd() {
  if (!SuperstructureTargeting.isPurgeZone(allianceConfirmed, currentZone)) {
    return Commands.idle().withName("Superstate(PURGE:DISALLOWED)");
  }
  return createPurgeMechanismCommand()
      .alongWith(stationaryAimCommand())
      .withName("Superstate(PURGE)");
}
```

The aimed autonomous version above continuously requests Intake `OUTTAKE` in purge zones instead of
the delayed Juicer sequence. The no-aim autonomous version never owns Drive.

- [ ] **Step 6: Run focused and drive regression tests**

Run the Step 3 command. Expected: all selected tests pass.

- [ ] **Step 7: Commit Drive/autonomous coordination**

```powershell
git add -- src/main/java/frc/robot/commands/DriveCommands.java src/main/java/frc/robot/subsystems/Superstructure.java src/test/java/frc/robot/commands/DriveCommandsTest.java src/test/java/frc/robot/subsystems/SuperstructureCommandRequirementsTest.java src/test/java/frc/robot/subsystems/SuperstructureAutonomousTest.java src/test/java/frc/robot/subsystems/SuperstructureSimulationTest.java
git diff --cached --check
git commit -m "feat(superstructure): add aiming and auto shooting"
```

---

### Task 6: Integrate Superstructure and named commands in RobotContainer

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Create: `src/test/java/frc/robot/RobotContainerSuperstructureTest.java`

**Interfaces:**
- Consumes: Tasks 3-5 public Superstructure API.
- Produces:
  - `private final Superstructure superstructure`
  - package-private `static void registerNamedCommands(Superstructure superstructure)`
  - default drive wired to Superstructure heading state
  - named commands available before `AutoBuilder.buildAutoChooser()`

- [ ] **Step 1: Write failing construction and registration tests**

```java
@Test
void namedCommandsRegisterTheFourRequiredSuperstructureCommands() {
  NamedCommands.clearAll();
  try {
    RobotContainer.registerNamedCommands(harness.superstructure);
    assertTrue(NamedCommands.hasCommand("Intake"));
    assertTrue(NamedCommands.hasCommand("Shoot"));
    assertTrue(NamedCommands.hasCommand("ShootNoAim"));
    assertTrue(NamedCommands.hasCommand("Drive"));
  } finally {
    NamedCommands.clearAll();
  }
}

@Test
void namedCommandRegistrationPrecedesChooserConstruction() {
  List<String> events = new ArrayList<>();
  RobotContainer.registerThenBuildChooser(
      () -> events.add("register"),
      () -> {
        events.add("chooser");
        return Commands.none();
      });
  assertEquals(List.of("register", "chooser"), events);
}
```

Add `registerThenBuildChooser(Runnable register, Supplier<T> chooserFactory)` as a package-private
generic ordering seam, following the existing `constructDriveThenVision` pattern.

```java
static <T> T registerThenBuildChooser(
    Runnable register, Supplier<T> chooserFactory) {
  Objects.requireNonNull(register).run();
  return Objects.requireNonNull(chooserFactory).get();
}
```

- [ ] **Step 2: Run the RobotContainer test and verify RED**

```powershell
.\gradlew.bat test --tests "frc.robot.RobotContainerSuperstructureTest" --console=plain --rerun-tasks --no-daemon
```

Expected: Superstructure field and registration helpers are absent.

- [ ] **Step 3: Construct Superstructure and reorder the chooser**

After Intake/Hopper/Shooter construction:

```java
superstructure = new Superstructure(drive, intake, hopper, shooter, vision);

autoChooser =
    registerThenBuildChooser(
        () -> registerNamedCommands(superstructure),
        () -> new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser()));
```

Implement exact named commands:

```java
static void registerNamedCommands(Superstructure superstructure) {
  NamedCommands.registerCommand(
      "Intake", superstructure.setStateCmd(Superstate.INTAKE));
  NamedCommands.registerCommand(
      "Shoot", superstructure.shootWithJuicerDelayCmd());
  NamedCommands.registerCommand(
      "ShootNoAim", superstructure.shootNoAimWithJuicerDelayCmd());
  NamedCommands.registerCommand(
      "Drive", superstructure.setStateCmd(Superstate.DRIVE));
}
```

Move `currentDriveHeading` and `rotationLastTriggered` out of RobotContainer. Pass Superstructure
getters/setters into `DriveCommands.joystickDefaultDrive` and use
`superstructure::getZoneLockedHeading` for zone lock. Remove RobotContainer's old fields and private
`getZoneLockedHeading()` helper, then remove imports made unused by that move.

- [ ] **Step 4: Run RobotContainer and existing construction tests**

```powershell
.\gradlew.bat test --tests "frc.robot.RobotContainerSuperstructureTest" --tests "frc.robot.RobotContainerVisionTest" --tests "frc.robot.RobotContainerIntakeTest" --tests "frc.robot.RobotContainerHopperTest" --tests "frc.robot.RobotContainerShooterTest" --console=plain --rerun-tasks --no-daemon
```

Expected: all construction, ordering, factory, and named-command tests pass.

- [ ] **Step 5: Commit container construction and autonomous registration**

```powershell
git add -- src/main/java/frc/robot/RobotContainer.java src/test/java/frc/robot/RobotContainerSuperstructureTest.java
git diff --cached --check
git commit -m "feat(robot): integrate superstructure commands"
```

---

### Task 7: Port driver and operator bindings

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Create: `src/test/java/frc/robot/RobotContainerSuperstructureBindingTest.java`

**Interfaces:**
- Consumes: Superstructure command factories, Drive heading reset, Shooter kicker override.
- Produces: package-private static
  `configureSuperstructureBindings(CommandXboxController driver, CommandXboxController operator, Drive drive, Intake intake, Hopper hopper, Shooter shooter, Superstructure superstructure)`.

- [ ] **Step 1: Write HID simulation tests for press and release behavior**

Initialize HAL and Driver Station simulation, create controllers on unused test ports, call the
binding helper, and poll the scheduler:

```java
@Test
void driverAUsesSafeConfigWhileHeldAndDriveOnRelease() {
  driverSim.setRawButton(XboxController.Button.kA.value, true);
  DriverStationSim.notifyNewData();
  scheduler.run();
  assertEquals(IntakeState.HOME, harness.intake.getDesiredState());

  driverSim.setRawButton(XboxController.Button.kA.value, false);
  DriverStationSim.notifyNewData();
  scheduler.run();
  assertEquals(Superstate.DRIVE, harness.superstructure.getCurrentState());
  assertEquals(IntakeState.DEPLOYED, harness.intake.getDesiredState());
}

@Test
void mechanismButtonsRestoreDriveOrDeployedOnRelease() {
  driverSim.setRawAxis(XboxController.Axis.kLeftTrigger.value, 1.0);
  DriverStationSim.notifyNewData();
  scheduler.run();
  assertEquals(IntakeState.INTAKE, harness.intake.getDesiredState());

  driverSim.setRawAxis(XboxController.Axis.kLeftTrigger.value, 0.0);
  DriverStationSim.notifyNewData();
  scheduler.run();
  assertEquals(Superstate.DRIVE, harness.superstructure.getCurrentState());

  driverSim.setRawButton(XboxController.Button.kB.value, true);
  DriverStationSim.notifyNewData();
  scheduler.run();
  assertEquals(IntakeState.JUICER, harness.intake.getDesiredState());

  driverSim.setRawButton(XboxController.Button.kB.value, false);
  DriverStationSim.notifyNewData();
  scheduler.run();
  assertEquals(IntakeState.DEPLOYED, harness.intake.getDesiredState());
}
```

Add cases for right-bumper outtake, right-trigger shot routing, operator reseed, both manual mode
toggles, operator Juicer, and kicker 100% duty. For kicker release, run one additional scheduler
cycle after the `runEnd` end action writes zero, then assert the Shooter state machine has resumed its
normal kicker output. Clear the scheduler's active button loop in `@AfterEach`.

- [ ] **Step 2: Run binding tests and verify RED**

```powershell
.\gradlew.bat test --tests "frc.robot.RobotContainerSuperstructureBindingTest" --console=plain --rerun-tasks --no-daemon
```

Expected: binding helper is absent.

- [ ] **Step 3: Implement the exact approved bindings**

Refactor the existing heading-reset and Vision-reseed bindings into this helper; do not leave the old
definitions in `configureButtonBindings()`. That method must install the Drive default and call
`configureSuperstructureBindings(...)` exactly once, preventing duplicate trigger callbacks.

```java
driver
    .leftTrigger(0.2)
    .onTrue(superstructure.setStateCmd(Superstate.INTAKE))
    .onFalse(superstructure.setStateCmd(Superstate.DRIVE));
driver
    .rightBumper()
    .onTrue(superstructure.setStateCmd(Superstate.OUTTAKE))
    .onFalse(superstructure.setStateCmd(Superstate.DRIVE));

Trigger shootTrigger = driver.rightTrigger(0.2);
shootTrigger
    .and(superstructure::shouldUsePurgeDuringShoot)
    .whileTrue(
        Commands.defer(
            superstructure::purgeShootCmd,
            Set.of(superstructure, drive, intake, hopper, shooter)));
shootTrigger
    .and(() -> !superstructure.shouldUsePurgeDuringShoot())
    .and(superstructure::isSelectedShootAllowed)
    .whileTrue(
        Commands.defer(
            superstructure::selectedShootModeCmd,
            Set.of(superstructure, drive, hopper, shooter)));
shootTrigger.onFalse(superstructure.setStateCmd(Superstate.DRIVE));

driver
    .b()
    .whileTrue(superstructure.intakeOverrideCmd(IntakeState.JUICER))
    .onFalse(superstructure.intakeOverrideCmd(IntakeState.DEPLOYED));
driver
    .a()
    .whileTrue(superstructure.setStateCmd(Superstate.DRIVE_STARTING_CONFIG))
    .onFalse(superstructure.setStateCmd(Superstate.DRIVE));
```

Add the remaining exact bindings:

```java
driver
    .start()
    .and(driver.back())
    .onTrue(
        Commands.runOnce(
                () -> {
                  drive.setPose(
                      new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
                  superstructure.setCurrentHeading(Optional.empty());
                },
                drive)
            .ignoringDisable(true)
            .withName("Driver Heading Reset"));

operator
    .start()
    .and(operator.back())
    .onTrue(superstructure.forceReseedFromVisionCmd());
operator
    .b()
    .and(operator.a().negate())
    .whileTrue(superstructure.intakeOverrideCmd(IntakeState.JUICER))
    .onFalse(superstructure.intakeOverrideCmd(IntakeState.DEPLOYED));
operator
    .x()
    .and(operator.a())
    .onTrue(
        superstructure.toggleShootModeCmd(ShootMode.MANUAL_BUMPER_UP));
operator
    .a()
    .and(operator.b())
    .onTrue(
        superstructure.toggleShootModeCmd(ShootMode.MANUAL_TRENCH));
operator
    .rightBumper()
    .whileTrue(
        Commands.runEnd(
                () -> shooter.runKickerPercentage(1.0),
                () -> shooter.runKickerPercentage(0.0),
                shooter)
            .withName("Kicker Full Power"));
```

- [ ] **Step 4: Run binding tests plus command ownership tests**

```powershell
.\gradlew.bat test --tests "frc.robot.RobotContainerSuperstructureBindingTest" --tests "frc.robot.subsystems.SuperstructureCommandRequirementsTest" --console=plain --rerun-tasks --no-daemon
```

Expected: every binding and requirement assertion passes.

- [ ] **Step 5: Commit controller bindings**

```powershell
git add -- src/main/java/frc/robot/RobotContainer.java src/test/java/frc/robot/RobotContainerSuperstructureBindingTest.java
git diff --cached --check
git commit -m "feat(robot): bind superstructure controls"
```

---

### Task 8: Add Hub Shift lifecycle, dashboard override, and complete logging

**Files:**
- Modify: `src/main/java/frc/robot/Robot.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Modify: `src/main/java/frc/robot/subsystems/Superstructure.java`
- Create: `src/test/java/frc/robot/RobotHubShiftLifecycleTest.java`
- Create: `src/test/java/frc/robot/subsystems/SuperstructureLoggingTest.java`

**Interfaces:**
- Consumes: `HubShiftUtil.initialize()`, official/shifted `ShiftInfo`, and Task 4 periodic state.
- Produces:
  - dashboard key `HubShift/WonAuto`
  - package-private `static Optional<Boolean> parseHubShiftOverride(String value)`
  - package-private installer seam for the dashboard-backed override supplier
  - package-private autonomous/teleop lifecycle helpers used directly by the overrides
  - `boolean isHubActive()`
  - `double getShiftTimeRemaining()`
  - complete `Superstructure/*` and `HubShift/*` AdvantageKit outputs

- [ ] **Step 1: Write failing lifecycle and logging tests**

Use package-private Robot helpers that contain the actual autonomous and teleop transition bodies,
then test both paths (including auto scheduling/cancellation), not just a bare Runnable wrapper:

```java
@Test
void lifecycleHelpersInitializeBothModesAndCancelTheAutoCommand() {
  AtomicInteger calls = new AtomicInteger();
  Command auto = Commands.idle().ignoringDisable(true);
  Command scheduled =
      Robot.startAutonomous(calls::incrementAndGet, () -> auto);
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
```

`RobotHubShiftLifecycleTest.@AfterEach` cancels scheduler commands, resets Driver Station data, and
restores `HubShiftUtil.setAllianceWinOverride(Optional::empty)` so static state cannot leak into later
tests.

Follow `ShooterLoggingTest`'s Logger reflection fixture and assert representative values plus stale
clearing:

```java
@Test
void periodicPublishesAndClearsTheCoordinationContract()
    throws ReflectiveOperationException {
  try (LoggerCapture capture = LoggerCapture.start()) {
    harness.setAlliance(AllianceStationID.Red1);
    harness.superstructure.periodic();
    assertEquals(
        "DRIVE_STARTING_CONFIG",
        capture.table().get("RealOutputs/Superstructure/CurrentState", ""));
    assertTrue(
        capture.table().get("RealOutputs/Superstructure/AllianceConfirmed", false));
    assertFalse(
        capture.table().get("RealOutputs/Superstructure/Zone", "").isBlank());
    assertEquals(
        FieldConstants.Hub.RED_CENTER_POSE,
        harness.superstructure.getCurrentAimTarget());

    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    harness.superstructure.periodic();
    assertFalse(
        capture.table().get("RealOutputs/Superstructure/AllianceConfirmed", true));
    assertEquals(
        "UNCONFIRMED",
        capture.table().get("RealOutputs/Superstructure/Zone", ""));
    assertEquals(
        FieldConstants.Hub.BLUE_CENTER_POSE,
        harness.superstructure.getCurrentAimTarget());

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
    assertTrue(capture.table().getAll(false).keySet().containsAll(requiredKeys));
  }
}
```

Implement `LoggerCapture` in the test file with the same saved/restored `Logger.running`,
`Logger.entry`, and `Logger.outputTable` fields used by existing mechanism logging tests.

```java
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
```

- [ ] **Step 2: Run lifecycle/logging tests and verify RED**

```powershell
.\gradlew.bat test --tests "frc.robot.RobotHubShiftLifecycleTest" --tests "frc.robot.subsystems.SuperstructureLoggingTest" --console=plain --rerun-tasks --no-daemon
```

Expected: lifecycle seam and log contract are absent.

- [ ] **Step 3: Add lifecycle resets and dashboard override**

```java
static Command startAutonomous(
    Runnable initializer, Supplier<Command> autonomousCommandSupplier) {
  Objects.requireNonNull(initializer).run();
  Command command = Objects.requireNonNull(autonomousCommandSupplier).get();
  if (command != null) command.schedule();
  return command;
}

static void startTeleop(Runnable initializer, Command autonomousCommand) {
  if (autonomousCommand != null) autonomousCommand.cancel();
  Objects.requireNonNull(initializer).run();
}

@Override
public void autonomousInit() {
  autonomousCommand =
      startAutonomous(HubShiftUtil::initialize, robotContainer::getAutonomousCommand);
}

@Override
public void teleopInit() {
  startTeleop(HubShiftUtil::initialize, autonomousCommand);
}
```

RobotContainer initializes `HubShift/WonAuto` to an empty string and installs this tested parser:

```java
static Optional<Boolean> parseHubShiftOverride(String value) {
  if (value == null) return Optional.empty();
  return switch (value.trim().toLowerCase(Locale.ROOT)) {
    case "true" -> Optional.of(true);
    case "false" -> Optional.of(false);
    default -> Optional.empty();
  };
}

static void installHubShiftOverride(
    Consumer<Supplier<Optional<Boolean>>> installer,
    Supplier<String> dashboardValue) {
  Supplier<String> checkedValue = Objects.requireNonNull(dashboardValue);
  Objects.requireNonNull(installer)
      .accept(() -> parseHubShiftOverride(checkedValue.get()));
}

private static void configureHubShiftOverride() {
  SmartDashboard.putString("HubShift/WonAuto", "");
  installHubShiftOverride(
      HubShiftUtil::setAllianceWinOverride,
      () ->
          SmartDashboard.getString("HubShift/WonAuto", ""));
}
```

Call `configureHubShiftOverride()` exactly once in the RobotContainer constructor immediately after
constructing Superstructure and before named-command registration/chooser construction. The installer
test above prevents the parser from existing without being connected to `HubShiftUtil`.

```java
superstructure = new Superstructure(drive, intake, hopper, shooter, vision);
configureHubShiftOverride();
autoChooser =
    registerThenBuildChooser(
        () -> registerNamedCommands(superstructure),
        () -> new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser()));
```

- [ ] **Step 4: Publish the complete AdvantageKit contract**

Record explicit values each loop:

```java
Logger.recordOutput("Superstructure/CurrentState", currentState.name());
Logger.recordOutput("Superstructure/ShootMode", shootMode.name());
Logger.recordOutput("Superstructure/AllianceConfirmed", allianceConfirmed);
Logger.recordOutput(
    "Superstructure/Alliance", allianceConfirmed ? alliance.name() : "UNCONFIRMED");
Logger.recordOutput(
    "Superstructure/Zone",
    allianceConfirmed && currentZone != null ? currentZone.name() : "UNCONFIRMED");
Logger.recordOutput("Superstructure/AimTarget", currentAimTarget);
Logger.recordOutput("Superstructure/DistanceToTargetMeters", distanceToTargetMeters);
Logger.recordOutput(
    "Superstructure/DistanceToTargetFeet",
    Units.metersToFeet(distanceToTargetMeters));
Logger.recordOutput("Superstructure/IsAlignedToTarget", alignedToTarget);
Logger.recordOutput(
    "Superstructure/ShooterReady",
    shooter.getCurrentState() == ShooterState.SHOOT);
Logger.recordOutput("Superstructure/FeedReady", isFeedReadyForCurrentState());
Logger.recordOutput(
    "Superstructure/ShootAllowed",
    SuperstructureTargeting.isShootAllowed(allianceConfirmed, currentZone));
Logger.recordOutput(
    "Superstructure/PurgeZone",
    SuperstructureTargeting.isPurgeZone(allianceConfirmed, currentZone));
```

Define the feed-ready log predicate explicitly:

```java
private boolean isFeedReadyForCurrentState() {
  boolean shooterReady = shooter.getCurrentState() == ShooterState.SHOOT;
  return switch (currentState) {
    case SHOOT_WITH_AIM, SHOOT_NO_AIM, PURGE -> shooterReady && alignedToTarget;
    case MANUAL_SHOOT -> shooterReady;
    default -> false;
  };
}
```

Read official and shifted `ShiftInfo` once per loop and log all four fields from each under their
approved prefixes, plus `HubShift/FirstActiveAlliance`. `isHubActive()` and
`getShiftTimeRemaining()` return shifted values. The logging test must assert the entire required-key
set above and the explicit `UNCONFIRMED` alliance/zone fallback after Driver Station data is reset.

- [ ] **Step 5: Run lifecycle, logging, and Hub Shift tests**

```powershell
.\gradlew.bat test --tests "frc.robot.RobotHubShiftLifecycleTest" --tests "frc.robot.subsystems.SuperstructureLoggingTest" --tests "frc.robot.util.HubShiftUtilTest" --console=plain --rerun-tasks --no-daemon
```

Expected: all selected tests pass.

- [ ] **Step 6: Commit lifecycle and telemetry**

```powershell
git add -- src/main/java/frc/robot/Robot.java src/main/java/frc/robot/RobotContainer.java src/main/java/frc/robot/subsystems/Superstructure.java src/test/java/frc/robot/RobotHubShiftLifecycleTest.java src/test/java/frc/robot/subsystems/SuperstructureLoggingTest.java
git diff --cached --check
git commit -m "feat(superstructure): add lifecycle telemetry"
```

---

### Task 9: Run full integration verification and review

**Files:**
- Verify all files changed in Tasks 1-8.
- Modify only files that Spotless or a failing test proves require correction.

**Interfaces:**
- Consumes the complete implementation.
- Produces a green, review-ready `feature/superstructure` branch with no unrelated changes.

- [ ] **Step 1: Run the complete focused superstructure slice**

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.Superstructure*" --tests "frc.robot.util.HubShiftUtilTest" --tests "frc.robot.RobotContainerSuperstructure*" --tests "frc.robot.RobotHubShiftLifecycleTest" --tests "frc.robot.commands.DriveCommandsTest" --console=plain --rerun-tasks --no-daemon
```

Expected: every selected test passes.

- [ ] **Step 2: Run full standard and isolated Vision verification**

```powershell
.\gradlew.bat check --console=plain --rerun-tasks --no-daemon
```

Expected: `BUILD SUCCESSFUL`, all standard tests pass, and all isolated `visionSimulationTest` tests
pass. PathPlanner named-command warnings for `Intake`, `Shoot`, `ShootNoAim`, and `Drive` must no
longer appear.

- [ ] **Step 3: Verify formatting and repository hygiene**

```powershell
git status --short
git diff --check
git diff --stat main...HEAD
git diff --name-status main...HEAD
```

Expected: no unstaged formatter changes, no whitespace errors, and only approved Superstructure,
Hub Shift, DriveCommands, RobotContainer, Robot, tests, spec, and plan files.

- [ ] **Step 4: Perform independent spec-compliance review**

Review `git diff main...HEAD` against
`docs/superpowers/specs/2026-07-31-superstructure-port-design.md`. Require explicit confirmation
that startup safety, every state mapping, zone restrictions, feed gates, requirement sets, 1.25
hood limit, manual presets, controller releases, named-command order, lifecycle resets, logging
clearing, and full verification evidence match the spec.

- [ ] **Step 5: Route any review finding through an exact TDD pair**

Use this fixed mapping so a finding cannot produce broad edits:

- targeting or zone finding: `SuperstructureTargetingTest.java` and
  `SuperstructureTargeting.java`;
- state, shooting, or autonomous finding: the corresponding
  `SuperstructureStateCommandTest.java`, `SuperstructureShootingTest.java`, or
  `SuperstructureAutonomousTest.java` plus `Superstructure.java`;
- coordinated physics finding: `SuperstructureSimulationTest.java` plus only the specific
  Superstructure or DriveCommands production file proven responsible;
- drive aiming finding: `DriveCommandsTest.java` plus `DriveCommands.java`;
- construction or binding finding: the corresponding
  `RobotContainerSuperstructureTest.java` or
  `RobotContainerSuperstructureBindingTest.java` plus `RobotContainer.java`;
- Hub Shift or lifecycle finding: `HubShiftUtilTest.java` or
  `RobotHubShiftLifecycleTest.java` plus `HubShiftUtil.java` or `Robot.java`;
- logging finding: `SuperstructureLoggingTest.java` plus `Superstructure.java`.

Add the regression assertion first, run its focused class to observe RED, make the minimal mapped
production edit, rerun GREEN, and commit only that mapped pair with
`fix(superstructure): correct reviewed behavior`. When review has no actionable finding, make no
additional commit.

- [ ] **Step 6: Re-run the final gate**

```powershell
.\gradlew.bat check --console=plain --rerun-tasks --no-daemon
git status --short
git diff --check
```

Expected: full build success and a clean worktree.
