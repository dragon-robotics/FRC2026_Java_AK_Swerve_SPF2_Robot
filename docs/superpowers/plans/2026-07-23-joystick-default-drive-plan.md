# Joystick Default Drive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task with verification checkpoints.

**Goal:** Port the reference default-drive behavior into the AdvantageKit drivetrain, including shaped inputs, half speed, heading hold, free rotation retention, zone lock, alliance handling, and focused tests.

**Architecture:** `DriveCommands` will own pure joystick shaping, mode selection, field-relative conversion, and the command loop. `Drive` will expose measured chassis angular velocity. `RobotContainer` will own heading/timestamp state and wire controller inputs plus a pose/alliance zone-heading supplier. Tests will exercise package-private pure helpers without constructing hardware IO.

**Tech Stack:** Java 17, WPILib 2026, AdvantageKit, WPILib command framework, JUnit 5, GradleRIO.

## Global Constraints

- Preserve existing user edits in `RobotContainer.java` and `DriveCommands.java` unless directly required by this feature.
- Keep existing 10% deadband for this command and square each joystick axis after deadband.
- Half-speed scales translation and rotation to exactly `0.45`, matching the reference command.
- Rotation retention window is `0.1` seconds; measured angular-speed threshold is `Math.toRadians(10)`.
- Use `Drive.runVelocity` and `ChassisSpeeds.fromFieldRelativeSpeeds`; do not introduce CTRE `SwerveRequest` APIs.
- Run tests red before production implementation, then focused tests and `./gradlew.bat testClasses`.

---

### Task 1: Add failing pure-behavior tests

**Files:**
- Create: `src/test/java/frc/robot/commands/DriveCommandsTest.java`

**Interfaces:**
- Tests consume package-private `DriveCommands.processJoystickInputs`, `DriveCommands.selectDriveMode`, `DriveCommands.toFieldRelativeSpeeds`, and `DriveCommands.getZoneLockedHeading`.
- Tests produce the executable behavior contract used by Task 2 and Task 3.

- [ ] **Step 1: Write tests for input shaping and half speed**

```java
@Test
void processJoystickInputsAppliesDeadbandSquaringAndHalfSpeed() {
  var full = DriveCommands.processJoystickInputs(0.5, -0.5, 0.5, false, 10.0, 4.0);
  var half = DriveCommands.processJoystickInputs(0.5, -0.5, 0.5, true, 10.0, 4.0);

  assertEquals(2.5, full.translation(), 1e-9);
  assertEquals(-2.5, full.strafe(), 1e-9);
  assertEquals(1.0, full.rotation(), 1e-9);
  assertEquals(1.125, half.translation(), 1e-9);
  assertEquals(-1.125, half.strafe(), 1e-9);
  assertEquals(0.45, half.rotation(), 1e-9);
}

@Test
void processJoystickInputsZerosValuesInsideDeadband() {
  var speeds = DriveCommands.processJoystickInputs(0.09, -0.09, 0.09, false, 10.0, 4.0);
  assertEquals(0.0, speeds.translation(), 1e-9);
  assertEquals(0.0, speeds.strafe(), 1e-9);
  assertEquals(0.0, speeds.rotation(), 1e-9);
}
```

- [ ] **Step 2: Write tests for mode precedence and field conversion**

```java
@Test
void selectDriveModeGivesZoneLockPrecedence() {
  assertEquals(
      DriveCommands.DriveMode.LOCK_ZONE,
      DriveCommands.selectDriveMode(true, true, true, true));
  assertEquals(
      DriveCommands.DriveMode.ROTATE,
      DriveCommands.selectDriveMode(false, true, true, true));
  assertEquals(
      DriveCommands.DriveMode.HOLD_HEADING,
      DriveCommands.selectDriveMode(false, false, false, false));
}

@Test
void toFieldRelativeSpeedsFlipsRobotHeadingForRedAlliance() {
  var blue = DriveCommands.toFieldRelativeSpeeds(
      new ChassisSpeeds(1.0, 0.0, 0.0), Rotation2d.kZero, Alliance.Blue);
  var red = DriveCommands.toFieldRelativeSpeeds(
      new ChassisSpeeds(1.0, 0.0, 0.0), Rotation2d.kZero, Alliance.Red);

  assertEquals(1.0, blue.vxMetersPerSecond, 1e-9);
  assertEquals(-1.0, red.vxMetersPerSecond, 1e-9);
}
```

- [ ] **Step 3: Write tests for zone heading policy**

```java
@Test
void getZoneLockedHeadingReturnsAllianceAdjustedLeftAndRightHeadings() {
  double fieldWidth = 8.0;
  assertEquals(-45.0, DriveCommands
      .getZoneLockedHeading(new Pose2d(1.0, 6.0, Rotation2d.kZero), Alliance.Blue, fieldWidth)
      .orElseThrow().getDegrees(), 1e-9);
  assertEquals(45.0, DriveCommands
      .getZoneLockedHeading(new Pose2d(1.0, 2.0, Rotation2d.kZero), Alliance.Blue, fieldWidth)
      .orElseThrow().getDegrees(), 1e-9);
  assertEquals(135.0, DriveCommands
      .getZoneLockedHeading(new Pose2d(1.0, 2.0, Rotation2d.kZero), Alliance.Red, fieldWidth)
      .orElseThrow().getDegrees(), 1e-9);
}
```

- [ ] **Step 4: Run tests and verify expected RED failure**

Run: `./gradlew.bat test --tests frc.robot.commands.DriveCommandsTest --console=plain`

Expected: compilation/test failure because the package-private helpers and `DriveMode` do not yet exist. Fix only test syntax/setup errors; do not add production behavior in this step.

### Task 2: Implement pure drive helpers and measured-speed access

**Files:**
- Modify: `src/main/java/frc/robot/commands/DriveCommands.java`
- Modify: `src/main/java/frc/robot/subsystems/drive/Drive.java`

**Interfaces:**
- `DriveCommands.processJoystickInputs(double, double, double, boolean, double, double)` returns `JoystickSpeeds` in meters/sec and radians/sec.
- `DriveCommands.selectDriveMode(boolean, boolean, boolean, boolean)` returns `DriveMode` with zone lock > rotate > hold heading precedence.
- `DriveCommands.toFieldRelativeSpeeds(ChassisSpeeds, Rotation2d, Alliance)` applies Red alliance rotation flip.
- `DriveCommands.getZoneLockedHeading(Pose2d, Alliance, double)` returns left/right lock heading based on alliance-normalized Y position.
- `Drive.getChassisSpeeds()` returns measured chassis speeds from existing module states.

- [ ] **Step 1: Add minimal helper types and methods**

Add package-private record `JoystickSpeeds(double translation, double strafe, double rotation)` and enum `DriveMode { LOCK_ZONE, ROTATE, HOLD_HEADING }`. Implement helpers exactly as specified by Task 1, using `MathUtil.applyDeadband(value, DEADBAND, 1.0)`, signed squaring, `0.45` half scaling, and `ChassisSpeeds.fromFieldRelativeSpeeds` with `driveRotation.plus(new Rotation2d(Math.PI))` for Red.

- [ ] **Step 2: Expose measured chassis speeds**

Change the existing private `getChassisSpeeds()` method in `Drive` to public. Keep its implementation and annotation unchanged.

- [ ] **Step 3: Run the focused tests and verify GREEN**

Run: `./gradlew.bat test --tests frc.robot.commands.DriveCommandsTest --console=plain`

Expected: all Task 1 tests pass.

### Task 3: Implement full command loop and container wiring

**Files:**
- Modify: `src/main/java/frc/robot/commands/DriveCommands.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java`

**Interfaces:**
- Preserve the existing `joystickDefaultDrive` signature.
- `RobotContainer` supplies mutable `Optional<Rotation2d>` heading state and `double` rotation timestamp to the command.
- Existing X-button angle command and B-button gyro reset remain intact; only the default command wiring changes.

- [ ] **Step 1: Replace incomplete default command body**

Create a `ProfiledPIDController` with existing `ANGLE_KP`, `ANGLE_KD`, `ANGLE_MAX_VELOCITY`, and `ANGLE_MAX_ACCELERATION`; enable continuous input. In each `Commands.run` cycle, read raw rotation, process inputs, calculate `rotationTriggered`, update the timestamp, calculate recent measured rotation using `drive.getChassisSpeeds().omegaRadiansPerSecond`, select mode, and then:

```java
switch (mode) {
  case LOCK_ZONE -> target = zoneHeading.get();
  case ROTATE -> {
    headingSetter.accept(Optional.empty());
    omega = speeds.rotation();
  }
  case HOLD_HEADING -> {
    if (headingGetter.get().isEmpty()) {
      headingSetter.accept(Optional.of(drive.getRotation()));
    }
    target = headingGetter.get();
  }
}
```

For `LOCK_ZONE` and `HOLD_HEADING`, calculate PID omega from current rotation to target. Convert the resulting chassis speeds with the helper and call `drive.runVelocity`. Reset the PID with current rotation in `.beforeStarting(...)`.

- [ ] **Step 2: Wire state, controls, and zone supplier**

Add `Optional<Rotation2d> currentDriveHeading = Optional.empty()` and `double rotationLastTriggered = 0.0` fields. Wire:

```java
DriveCommands.joystickDefaultDrive(
    drive,
    () -> -controller.getLeftY(),
    () -> -controller.getLeftX(),
    () -> -controller.getRightX(),
    () -> controller.getHID().getPOV() == 0,
    () -> controller.getHID().getXButton(),
    () -> currentDriveHeading,
    heading -> currentDriveHeading = heading,
    () -> rotationLastTriggered,
    timestamp -> rotationLastTriggered = timestamp,
    () -> DriveCommands.getZoneLockedHeading(
        drive.getPose(),
        DriverStation.getAlliance().orElse(Alliance.Blue),
        FIELD_WIDTH_METERS));
```

Use the default 2026 field width from the WPILib AprilTag field layout in a private container constant. Keep existing A/B/X bindings unchanged.

- [ ] **Step 3: Run focused tests and compile**

Run: `./gradlew.bat test --tests frc.robot.commands.DriveCommandsTest --console=plain` and then `./gradlew.bat testClasses --console=plain`.

Expected: focused tests pass; `testClasses` exits 0. If unrelated baseline tests fail, report their names separately.

### Task 4: Review diff and verification state

**Files:**
- Review: `src/main/java/frc/robot/commands/DriveCommands.java`
- Review: `src/main/java/frc/robot/subsystems/drive/Drive.java`
- Review: `src/main/java/frc/robot/RobotContainer.java`
- Review: `src/test/java/frc/robot/commands/DriveCommandsTest.java`

- [ ] **Step 1: Run formatting and whitespace checks**

Run: `./gradlew.bat spotlessApply --console=plain`, then `git diff --check`.

- [ ] **Step 2: Re-run focused verification after formatting**

Run: `./gradlew.bat test --tests frc.robot.commands.DriveCommandsTest --console=plain` and `./gradlew.bat testClasses --console=plain`.

- [ ] **Step 3: Inspect status and diff scope**

Run: `git status --short` and `git diff --stat`. Confirm only the approved spec/plan plus drive implementation and focused test files changed; do not stage or revert unrelated user edits.
