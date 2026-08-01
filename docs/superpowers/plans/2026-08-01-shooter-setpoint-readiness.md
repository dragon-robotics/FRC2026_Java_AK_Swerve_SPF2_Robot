# Shooter Setpoint Readiness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Shooter use a tight acquisition window and a downward-biased maintenance window while preserving Hopper feeding across satisfied setpoint refreshes.

**Architecture:** Shooter owns phase-specific readiness and immediately returns to transition outputs when maintenance readiness is lost. Superstructure continues to gate Hopper on `ShooterState.SHOOT`, but sends each neutral-zone flywheel/hood target to Shooter atomically so an intermediate hood target cannot create a false transition.

**Tech Stack:** Java 17, WPILib command scheduler, AdvantageKit, JUnit 5, GradleRIO

## Global Constraints

- Enter `SHOOT` only at flywheel target +/-60 RPM and hood target +/-0.125 rotations.
- Maintain `SHOOT` at flywheel target -120/+60 RPM and hood target +/-0.125 rotations.
- All tolerance bounds are inclusive.
- Losing maintenance readiness immediately requests `TRANSITION` behavior and 6 V kicker preparation output.
- Hopper remains gated by `shooter.getCurrentState() == ShooterState.SHOOT` and the existing alignment rules.
- Neutral-zone targets preserve interpolated RPM and the existing 1.25-rotation hood override.
- Do not change motor configuration, current limits, PID values, control modes, alignment rules, or target values.

---

## File Map

- `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`: names the acquisition, maintenance, and prep flywheel windows.
- `src/main/java/frc/robot/subsystems/shooter/Shooter.java`: evaluates phase-specific readiness, preserves satisfied `SHOOT` state across target refreshes, and immediately falls back to transition outputs.
- `src/test/java/frc/robot/subsystems/shooter/ShooterTest.java`: protects readiness boundaries, state transitions, and kicker behavior.
- `src/main/java/frc/robot/subsystems/Superstructure.java`: selects one final distance/hood target per periodic call.
- `src/test/java/frc/robot/subsystems/SuperstructureShootingTest.java`: reproduces the full scheduler-loop Hopper feed failure.

### Task 1: Phase-Specific Shooter Readiness

**Files:**
- Modify: `src/test/java/frc/robot/subsystems/shooter/ShooterTest.java:112-270`
- Modify: `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java:35-41`
- Modify: `src/main/java/frc/robot/subsystems/shooter/Shooter.java:52-207`

**Interfaces:**
- Consumes: existing `Shooter.setSetpoint(double, double)`, `Shooter.setDesiredState(ShooterState)`, and `Shooter.periodic()`.
- Produces: phase-aware `Shooter.isFlywheelReady()` and unchanged public state/setpoint APIs.

- [ ] **Step 1: Tighten the acquisition regression and add maintenance regressions**

Update `shootWaitsForFlywheelAndHoodThenFeeds()` so the first two samples independently fail the
lower flywheel and hood acquisition boundaries, then accept the inclusive lower boundaries:

```java
io.measuredFlywheelRpm = 2439.99;
io.measuredHoodRotations = 0.75;
shooter.periodic();
assertEquals(ShooterState.TRANSITION, shooter.getCurrentState());
assertEquals(6.0, io.kickerVolts, 1e-9);

io.measuredFlywheelRpm = 2440.0;
io.measuredHoodRotations = 0.6249;
shooter.periodic();
assertEquals(ShooterState.TRANSITION, shooter.getCurrentState());

io.measuredHoodRotations = 0.625;
shooter.periodic();
assertEquals(ShooterState.SHOOT, shooter.getCurrentState());
assertEquals(12.0, io.kickerVolts, 1e-9);
```

Replace the old always-asymmetric readiness test and add these behavior tests:

```java
@Test
void shootAcquisitionUsesInclusiveSymmetricUpperBound() {
  RecordingIO io = new RecordingIO();
  Shooter shooter = new Shooter(io);
  shooter.periodic();
  shooter.setSetpoint(2500.0, 0.75);
  shooter.setDesiredState(ShooterState.SHOOT);
  io.measuredHoodRotations = 0.75;
  io.measuredFlywheelRpm = 2560.01;
  shooter.periodic();
  assertEquals(ShooterState.TRANSITION, shooter.getCurrentState());
  io.measuredFlywheelRpm = 2560.0;
  shooter.periodic();
  assertEquals(ShooterState.SHOOT, shooter.getCurrentState());
}

@Test
void shootMaintenanceUsesInclusiveBiasedWindowAndImmediatePrepFallback() {
  RecordingIO io = new RecordingIO();
  Shooter shooter = new Shooter(io);
  shooter.periodic();
  shooter.setSetpoint(2500.0, 0.75);
  shooter.setDesiredState(ShooterState.SHOOT);
  io.measuredFlywheelRpm = 2500.0;
  io.measuredHoodRotations = 0.75;
  shooter.periodic();

  io.measuredFlywheelRpm = 2380.0;
  shooter.periodic();
  assertEquals(ShooterState.SHOOT, shooter.getCurrentState());
  assertEquals(12.0, io.kickerVolts, 1e-9);
  io.measuredFlywheelRpm = 2560.0;
  shooter.periodic();
  assertEquals(ShooterState.SHOOT, shooter.getCurrentState());

  io.measuredFlywheelRpm = 2379.99;
  shooter.periodic();
  assertEquals(ShooterState.TRANSITION, shooter.getCurrentState());
  assertEquals(6.0, io.kickerVolts, 1e-9);

  io.measuredFlywheelRpm = 2500.0;
  shooter.periodic();
  assertEquals(ShooterState.SHOOT, shooter.getCurrentState());
  io.measuredFlywheelRpm = 2560.01;
  shooter.periodic();
  assertEquals(ShooterState.TRANSITION, shooter.getCurrentState());
  assertEquals(6.0, io.kickerVolts, 1e-9);
}

@Test
void shootMaintenanceUsesHoodToleranceAndImmediatePrepFallback() {
  RecordingIO io = new RecordingIO();
  Shooter shooter = new Shooter(io);
  shooter.periodic();
  shooter.setSetpoint(2500.0, 0.75);
  shooter.setDesiredState(ShooterState.SHOOT);
  io.measuredFlywheelRpm = 2500.0;
  io.measuredHoodRotations = 0.75;
  shooter.periodic();

  io.measuredHoodRotations = 0.875;
  shooter.periodic();
  assertEquals(ShooterState.SHOOT, shooter.getCurrentState());
  io.measuredHoodRotations = 0.8751;
  shooter.periodic();
  assertEquals(ShooterState.TRANSITION, shooter.getCurrentState());
  assertEquals(6.0, io.kickerVolts, 1e-9);
}

@Test
void satisfiedSetpointRefreshUsesMaintenanceWindowWithoutReacquiring() {
  RecordingIO io = new RecordingIO();
  Shooter shooter = new Shooter(io);
  shooter.periodic();
  shooter.setSetpoint(2500.0, 0.75);
  shooter.setDesiredState(ShooterState.SHOOT);
  io.measuredFlywheelRpm = 2500.0;
  io.measuredHoodRotations = 0.75;
  shooter.periodic();

  shooter.setSetpoint(2580.0, 0.80);
  assertEquals(ShooterState.SHOOT, shooter.getCurrentState());
  shooter.periodic();
  assertEquals(ShooterState.SHOOT, shooter.getCurrentState());
  assertEquals(12.0, io.kickerVolts, 1e-9);
}
```

Keep `changedShootSetpointReturnsToTransitionUntilNewTargetIsReady()` unchanged to protect large
target changes.

- [ ] **Step 2: Run the Shooter tests and confirm the new expectations fail**

Run:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.shooter.ShooterTest" --console=plain --rerun-tasks --no-daemon
```

Expected: FAIL because acquisition currently accepts target -120 RPM, `SHOOT` never monitors the
maintenance window, and every changed shooting setpoint forces `TRANSITION`.

- [ ] **Step 3: Implement phase-specific readiness and immediate fallback**

Replace the generic readiness constants with semantically named values in `ShooterConstants`:

```java
static final double SHOOT_ENTRY_TOLERANCE_RPM = 60.0;
static final double SHOOT_MAINTENANCE_BELOW_RPM = 120.0;
static final double SHOOT_MAINTENANCE_ABOVE_RPM = 60.0;
static final double PREP_READY_BELOW_RPM = 120.0;
static final double PREP_READY_ABOVE_RPM = 60.0;
```

In `Shooter`, capture whether the mechanism was already shooting before assigning a changed target:

```java
public void setSetpoint(double flywheelRpm, double hoodRotations) {
  boolean changed = targetRpm != flywheelRpm || targetHoodRotations != hoodRotations;
  boolean wasShooting = currentState == ShooterState.SHOOT;
  targetRpm = flywheelRpm;
  targetHoodRotations = hoodRotations;
  if (changed && desiredState == ShooterState.SHOOT) {
    currentState =
        wasShooting && isShootMaintenanceReady()
            ? ShooterState.SHOOT
            : ShooterState.TRANSITION;
  }
}
```

Make public readiness phase-aware and split the private predicates:

```java
public boolean isFlywheelReady() {
  return currentState == ShooterState.SHOOT
      ? isFlywheelWithin(
          targetRpm, SHOOT_MAINTENANCE_BELOW_RPM, SHOOT_MAINTENANCE_ABOVE_RPM)
      : isFlywheelWithin(
          targetRpm, SHOOT_ENTRY_TOLERANCE_RPM, SHOOT_ENTRY_TOLERANCE_RPM);
}

private boolean isShootEntryReady() {
  return isFlywheelWithin(
          targetRpm, SHOOT_ENTRY_TOLERANCE_RPM, SHOOT_ENTRY_TOLERANCE_RPM)
      && isHoodReady();
}

private boolean isShootMaintenanceReady() {
  return isFlywheelWithin(
          targetRpm, SHOOT_MAINTENANCE_BELOW_RPM, SHOOT_MAINTENANCE_ABOVE_RPM)
      && isHoodReady();
}

private boolean isFlywheelWithin(double requestedRpm, double belowRpm, double aboveRpm) {
  double actualRpm = inputs.flywheelLeadVelocityRpm;
  return actualRpm >= requestedRpm - belowRpm && actualRpm <= requestedRpm + aboveRpm;
}
```

Route `SHOOT` through a maintenance method and share transition outputs:

```java
case SHOOT -> maintainShoot();

private void transitionToShoot() {
  if (isShootEntryReady()) {
    commandShoot();
    currentState = ShooterState.SHOOT;
  } else {
    commandShootTransition();
  }
}

private void maintainShoot() {
  if (isShootMaintenanceReady()) {
    commandShoot();
  } else {
    commandShootTransition();
    currentState = ShooterState.TRANSITION;
  }
}

private void commandShootTransition() {
  runFlywheelVelocity(targetRpm);
  runHoodPosition(targetHoodRotations);
  runKickerVoltage(KICKER_PREP_VOLTAGE);
}
```

Update `transitionToPrepFuel()` to call `isFlywheelWithin(PREP_FLYWHEEL_RPM,
PREP_READY_BELOW_RPM, PREP_READY_ABOVE_RPM)` so PREPFUEL behavior remains unchanged.

- [ ] **Step 4: Run focused Shooter verification**

Run the Step 2 command again.

Expected: PASS, including the existing large-setpoint-change and PREPFUEL regressions.

- [ ] **Step 5: Commit the phase-specific Shooter behavior**

```powershell
git add src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java src/main/java/frc/robot/subsystems/shooter/Shooter.java src/test/java/frc/robot/subsystems/shooter/ShooterTest.java
git commit -m "fix(shooter): separate acquisition and maintenance readiness"
```

### Task 2: Atomic Neutral-Zone Setpoint Refresh

**Files:**
- Modify: `src/test/java/frc/robot/subsystems/SuperstructureShootingTest.java:108-130`
- Modify: `src/main/java/frc/robot/subsystems/shooter/Shooter.java:76-83`
- Modify: `src/main/java/frc/robot/subsystems/Superstructure.java:391-415`

**Interfaces:**
- Consumes: Task 1's readiness-aware `Shooter.setSetpoint(double, double)` behavior.
- Produces: `Shooter.setSetpointForDistance(double distanceMeters, double hoodRotations)` for an atomic interpolated-RPM/fixed-hood target.

- [ ] **Step 1: Add a scheduler-level regression for neutral-zone feeding**

Add to `SuperstructureShootingTest`:

```java
@Test
void readyShooterFeedsAcrossNeutralZoneTargetRefresh() {
  harness.setAlliance(AllianceStationID.Blue1);
  Pose2d unrotated = harness.poseInside(FieldZones.NEUTRAL_LEFT_SHOOT);
  Translation2d target = FieldConstants.AimPoints.BLUE_LEFT_SHOOT_POINT;
  Pose2d aligned =
      new Pose2d(unrotated.getTranslation(), target.minus(unrotated.getTranslation()).getAngle());
  harness.setPose(aligned);
  assertTrue(harness.superstructure.isAlignedToTarget());

  Command command = harness.superstructure.setStateCmd(Superstate.SHOOT_NO_AIM);
  harness.run(command);
  harness.makeShooterReady();
  assertEquals(ShooterState.SHOOT, harness.shooter.getCurrentState());

  CommandScheduler.getInstance().run();

  assertEquals(ShooterState.SHOOT, harness.shooter.getCurrentState());
  assertEquals(HopperState.INDEX_TO_SHOOTER, harness.hopper.getDesiredState());
}
```

- [ ] **Step 2: Run the integration test and verify the reproduced failure**

Run:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.SuperstructureShootingTest.readyShooterFeedsAcrossNeutralZoneTargetRefresh" --console=plain --rerun-tasks --no-daemon
```

Expected: FAIL because Superstructure applies the interpolated hood and then the 1.25-rotation
override as two target changes, leaving Shooter in `TRANSITION` before the Hopper feed check.

- [ ] **Step 3: Add the atomic distance/hood overload**

Add to `Shooter` without changing the existing overload:

```java
public void setSetpointForDistance(double distanceMeters, double hoodRotations) {
  ShooterSetpoint setpoint = ShooterConstants.getSetpointForDistance(distanceMeters);
  setSetpoint(setpoint.flywheelRpm(), hoodRotations);
}
```

Change the Superstructure targeting branch to call exactly one setter:

```java
if (currentState != Superstate.MANUAL_SHOOT) {
  if (SuperstructureTargeting.isNeutralShootOrPurgeZone(allianceConfirmed, currentZone)) {
    shooter.setSetpointForDistance(distanceToTargetMeters, NEUTRAL_ZONE_HOOD_ROTATIONS);
  } else {
    shooter.setSetpointForDistance(distanceToTargetMeters);
  }
}
```

- [ ] **Step 4: Run focused Shooter and Superstructure verification**

Run:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.shooter.ShooterTest" --tests "frc.robot.subsystems.SuperstructureShootingTest" --console=plain --rerun-tasks --no-daemon
```

Expected: PASS. Confirm the existing four-zone hood-cap test still reports interpolated RPM and
exactly 1.25 hood rotations.

- [ ] **Step 5: Commit the atomic Superstructure target update**

```powershell
git add src/main/java/frc/robot/subsystems/shooter/Shooter.java src/main/java/frc/robot/subsystems/Superstructure.java src/test/java/frc/robot/subsystems/SuperstructureShootingTest.java
git commit -m "fix(superstructure): apply shooter targets atomically"
```

### Task 3: Full Verification and Diff Review

**Files:**
- Verify: all files changed by Tasks 1 and 2

**Interfaces:**
- Consumes: completed Task 1 and Task 2 commits.
- Produces: a verified branch ready for review; no new production API.

- [ ] **Step 1: Run formatting and the full test/check gate**

```powershell
.\gradlew.bat spotlessApply check --console=plain --no-daemon
```

Expected: `BUILD SUCCESSFUL` with no failing tests.

- [ ] **Step 2: Re-run focused tests if formatting changed Java sources**

```powershell
git status --short
.\gradlew.bat test --tests "frc.robot.subsystems.shooter.ShooterTest" --tests "frc.robot.subsystems.SuperstructureShootingTest" --console=plain --rerun-tasks --no-daemon
```

Expected: focused tests pass against the formatted files.

- [ ] **Step 3: Inspect repository hygiene and the exact branch diff**

```powershell
git diff --check
git diff --stat HEAD~2..HEAD
git log -3 --oneline
git status --short --branch
```

Expected: no whitespace errors, only the approved Shooter/Superstructure behavior and regressions,
and a clean worktree. If Spotless changed tracked files, stage only the in-scope formatting and
amend the corresponding implementation commit after reviewing the patch.
