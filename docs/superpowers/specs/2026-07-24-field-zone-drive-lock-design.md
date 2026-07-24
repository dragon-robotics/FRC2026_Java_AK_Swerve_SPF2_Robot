# Field Zone Drive Lock Design

## Goal

Replace the primitive field-width split used by default-drive angle lock with the checked-in `FieldConstants.FieldZones` classifier.

## Architecture

`RobotContainer` will classify the current drivetrain pose with `FieldZones.fromPose(pose, alliance)` whenever the default drive command requests a zone-locked heading. `DriveCommands` will accept the resulting `FieldZones` value and map only supported zones to alliance-aware lock headings.

This keeps field geometry and pose classification in `FieldConstants`, while `DriveCommands` remains responsible only for translating a classified zone into a drive heading.

## Zone Mapping

The mapping matches the reference robot behavior:

- `ALLIANCE_LEFT`, `NEUTRAL_LEFT_SHOOT`, `NEUTRAL_LEFT_PURGE`, `NEUTRAL_LEFT`, and `OPPONENT_LEFT` lock to -45 degrees on Blue and 135 degrees on Red.
- `ALLIANCE_RIGHT`, `NEUTRAL_RIGHT_SHOOT`, `NEUTRAL_RIGHT_PURGE`, `NEUTRAL_RIGHT`, and `OPPONENT_RIGHT` lock to 45 degrees on Blue and -135 degrees on Red.
- Trench and bump zones return `Optional.empty()` and do not engage zone lock.

## Field Constants Compatibility

The local `FieldConstants.java` declares package `frc.robot` and remains at its current path. Its zone classifier still uses the old `SwerveConstants` class name. Import `frc.robot.subsystems.drive.DriveConstants` and replace those references with the existing `DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS` and `DriveConstants.ROBOT_CENTER_TO_CORNER_WITH_BUMPERS_METERS` values.

No footprint constants will be duplicated inside `FieldConstants`.

## Code Changes

- `FieldConstants.java`: replace obsolete `SwerveConstants` references with existing `DriveConstants` footprint values.
- `DriveCommands.java`: replace `getZoneLockedHeading(Pose2d, Alliance, double)` with `getZoneLockedHeading(FieldZones, Alliance)`.
- `RobotContainer.java`: remove the local field-width constant and classify the pose with `FieldZones.fromPose` before requesting a heading.
- `DriveCommandsTest.java`: test left/right mapping and confirm trench/bump zones return empty.

## Verification

Use TDD: update the tests first and confirm the old primitive API fails. Then implement the zone-based API, run the focused `DriveCommandsTest`, run `testClasses`, and check the final diff for unrelated Spotless changes.
