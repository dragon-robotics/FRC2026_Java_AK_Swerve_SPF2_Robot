# Field Zone Drive Lock Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace default-drive's primitive field-width split with `FieldConstants.FieldZones.fromPose(...)` and the reference zone-to-heading mapping.

**Architecture:** `FieldConstants` owns pose-to-zone classification and uses existing `DriveConstants` footprint dimensions. `RobotContainer` obtains the current zone, while `DriveCommands` maps the classified zone and alliance to an optional heading. Tests cover classification symmetry plus lockable and excluded zone mappings.

**Tech Stack:** Java 17, WPILib 2026, AdvantageKit, JUnit 5, GradleRIO.

## Global Constraints

- Preserve all unrelated dirty files, including the user's ongoing `TunerConstants` and drive-constant relocation.
- Use `DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS` and `DriveConstants.ROBOT_CENTER_TO_CORNER_WITH_BUMPERS_METERS`; do not duplicate footprint values.
- Bump and trench zones must return `Optional.empty()`.
- Run Gradle verification with `-x spotlessApply` so the build does not rewrite unrelated dirty Java files.

---

### Task 1: Add failing field-zone behavior tests

**Files:**
- Modify: `src/test/java/frc/robot/commands/DriveCommandsTest.java`
- Create: `src/test/java/frc/robot/FieldConstantsTest.java`

**Interfaces:**
- Consumes: desired `DriveCommands.getZoneLockedHeading(FieldConstants.FieldZones, DriverStation.Alliance)` API.
- Produces: regression coverage for zone classification, alliance heading mapping, and excluded zones.

- [ ] **Step 1: Replace primitive heading tests with zone mapping tests**

Use representative enum values:

```java
assertEquals(
    -45.0,
    DriveCommands.getZoneLockedHeading(FieldZones.ALLIANCE_LEFT, Alliance.Blue)
        .orElseThrow()
        .getDegrees(),
    1e-9);
assertEquals(
    45.0,
    DriveCommands.getZoneLockedHeading(FieldZones.NEUTRAL_RIGHT, Alliance.Blue)
        .orElseThrow()
        .getDegrees(),
    1e-9);
assertEquals(
    135.0,
    DriveCommands.getZoneLockedHeading(FieldZones.OPPONENT_LEFT, Alliance.Red)
        .orElseThrow()
        .getDegrees(),
    1e-9);
assertEquals(
    -135.0,
    DriveCommands.getZoneLockedHeading(FieldZones.NEUTRAL_RIGHT_PURGE, Alliance.Red)
        .orElseThrow()
        .getDegrees(),
    1e-9);
assertTrue(DriveCommands.getZoneLockedHeading(FieldZones.ALLIANCE_LEFT_TRENCH, Alliance.Blue).isEmpty());
assertTrue(DriveCommands.getZoneLockedHeading(FieldZones.OPPONENT_RIGHT_BUMP, Alliance.Red).isEmpty());
```

- [ ] **Step 2: Add field classification tests**

Create `FieldConstantsTest` with alliance-zone poses that avoid boundary ambiguity:

```java
@Test
void fieldZonesClassifiesAllianceLeftAndRightFromPose() {
  assertEquals(
      FieldZones.ALLIANCE_LEFT,
      FieldZones.fromPose(
          new Pose2d(0.0, FieldConstants.FIELD_WIDTH * 0.75, Rotation2d.kZero), Alliance.Blue));
  assertEquals(
      FieldZones.ALLIANCE_RIGHT,
      FieldZones.fromPose(
          new Pose2d(0.0, FieldConstants.FIELD_WIDTH * 0.25, Rotation2d.kZero), Alliance.Blue));
}

@Test
void fieldZonesMirrorsRedAlliancePoseBeforeClassification() {
  assertEquals(
      FieldZones.ALLIANCE_LEFT,
      FieldZones.fromPose(
          new Pose2d(
              FieldConstants.FIELD_LENGTH,
              FieldConstants.FIELD_WIDTH * 0.25,
              Rotation2d.kZero),
          Alliance.Red));
}
```

- [ ] **Step 3: Run focused tests and verify RED**

Run: `./gradlew.bat test --tests frc.robot.commands.DriveCommandsTest --tests frc.robot.FieldConstantsTest -x spotlessApply --console=plain`

Expected: compile failure because the old `SwerveConstants` references and primitive `getZoneLockedHeading(Pose2d, Alliance, double)` API remain.

### Task 2: Connect FieldConstants to DriveConstants

**Files:**
- Modify: `src/main/java/frc/robot/FieldConstants.java`

**Interfaces:**
- Consumes: `frc.robot.subsystems.drive.DriveConstants` footprint constants.
- Produces: compiling `FieldZones.fromPose(Pose2d, Alliance)` classification.

- [ ] **Step 1: Import DriveConstants and replace obsolete references**

Add:

```java
import frc.robot.subsystems.drive.DriveConstants;
```

Replace every `SwerveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS` with `DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS`, and every `SwerveConstants.ROBOT_CENTER_TO_CORNER_WITH_BUMPERS_METERS` with `DriveConstants.ROBOT_CENTER_TO_CORNER_WITH_BUMPERS_METERS`.

- [ ] **Step 2: Compile field classification tests**

Run: `./gradlew.bat test --tests frc.robot.FieldConstantsTest -x spotlessApply --console=plain`

Expected: `FieldConstantsTest` passes.

### Task 3: Replace primitive heading selection

**Files:**
- Modify: `src/main/java/frc/robot/commands/DriveCommands.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Test: `src/test/java/frc/robot/commands/DriveCommandsTest.java`

**Interfaces:**
- Consumes: `FieldZones.fromPose(Pose2d, Alliance)`.
- Produces: `public static Optional<Rotation2d> getZoneLockedHeading(FieldZones zone, Alliance alliance)`.

- [ ] **Step 1: Implement exact zone-to-heading mapping**

Use one switch in `DriveCommands`:

```java
double leftLockDegrees = alliance == Alliance.Red ? 135.0 : -45.0;
double rightLockDegrees = alliance == Alliance.Red ? -135.0 : 45.0;

return switch (zone) {
  case ALLIANCE_LEFT,
      NEUTRAL_LEFT_SHOOT,
      NEUTRAL_LEFT_PURGE,
      NEUTRAL_LEFT,
      OPPONENT_LEFT -> Optional.of(Rotation2d.fromDegrees(leftLockDegrees));
  case ALLIANCE_RIGHT,
      NEUTRAL_RIGHT_SHOOT,
      NEUTRAL_RIGHT_PURGE,
      NEUTRAL_RIGHT,
      OPPONENT_RIGHT -> Optional.of(Rotation2d.fromDegrees(rightLockDegrees));
  default -> Optional.empty();
};
```

- [ ] **Step 2: Classify pose in RobotContainer**

Remove `AprilTagFieldLayout`, `AprilTagFields`, and `FIELD_WIDTH_METERS`. Implement:

```java
private Optional<Rotation2d> getZoneLockedHeading() {
  return DriverStation.getAlliance()
      .flatMap(
          alliance ->
              DriveCommands.getZoneLockedHeading(
                  FieldZones.fromPose(drive.getPose(), alliance), alliance));
}
```

- [ ] **Step 3: Run focused tests and verify GREEN**

Run: `./gradlew.bat test --tests frc.robot.commands.DriveCommandsTest --tests frc.robot.FieldConstantsTest -x spotlessApply --console=plain`

Expected: all focused tests pass.

### Task 4: Verify scope and compilation

**Files:**
- Review: `src/main/java/frc/robot/FieldConstants.java`
- Review: `src/main/java/frc/robot/commands/DriveCommands.java`
- Review: `src/main/java/frc/robot/RobotContainer.java`
- Review: `src/test/java/frc/robot/FieldConstantsTest.java`
- Review: `src/test/java/frc/robot/commands/DriveCommandsTest.java`

- [ ] **Step 1: Compile all tests without Spotless mutations**

Run: `./gradlew.bat testClasses -x spotlessApply --console=plain`

Expected: build succeeds with no compile errors.

- [ ] **Step 2: Run whitespace and diff checks**

Run: `git diff --check`, `git status --short`, and `git diff --stat`.

Expected: no whitespace errors; unrelated dirty files retain their prior content and status.
