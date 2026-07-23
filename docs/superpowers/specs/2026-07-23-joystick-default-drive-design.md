# Joystick Default Drive Design

## Goal

Port the full behavior of the reference `DefaultDriveCmd` into this AdvantageKit drivetrain as `DriveCommands.joystickDefaultDrive`, while keeping the existing `Drive.runVelocity` and field-relative control path.

## Behavior

`joystickDefaultDrive` will consume translation, strafe, rotation, half-speed, angle-lock, heading-state, rotation-activity-state, and zone-heading suppliers. Each scheduler cycle it will:

1. Apply the existing 10% joystick deadband and square translation and rotation magnitudes.
2. Scale translation by `Drive.getMaxLinearSpeedMetersPerSec()` and rotation by `Drive.getMaxAngularSpeedRadPerSec()`.
3. Scale all three axes to 45% while half-speed is active, matching the reference command.
4. Treat rotation as active when the raw rotation input exceeds the deadband, or when a recent rotation trigger (within 0.1 seconds) is paired with measured angular velocity above 10 degrees per second.
5. Prefer zone heading lock when angle lock is held and the zone supplier provides a heading.
6. Permit free rotation while rotation is active and clear stored heading during that mode.
7. Otherwise capture the current field heading once and maintain it with a continuous-input profiled PID controller.
8. Convert field-relative speeds using the current robot heading, adding 180 degrees for Red alliance, then send them through `Drive.runVelocity`.

The heading PID resets when the command starts. Existing joystick sign conventions remain unchanged. POV-up supplies half-speed and X supplies angle lock. The zone-heading supplier remains injectable so field-zone policy does not become coupled to drivetrain command code; the local container provides the current left/right lock policy from pose and alliance.

## Boundaries

- `DriveCommands` owns joystick shaping, rotation mode selection, heading PID, and chassis-speed generation.
- `RobotContainer` owns mutable default-drive state and wires controller inputs plus the zone-heading supplier.
- No CTRE `SwerveRequest`, `CommandSwerveDrivetrain`, Superstructure, shooter, intake, or unrelated field behavior is ported.
- Existing unrelated working-tree edits remain untouched.

## Testing

Add focused tests for the pure drive-input shaping and command policy. Required cases:

- deadband and squared input scaling;
- 45% half-speed scaling on translation and rotation;
- rotation trigger timestamp update and recent-rotation retention;
- heading capture when rotation is inactive;
- heading clearing during free rotation;
- zone-lock precedence when a zone heading is available;
- Red-alliance field-relative conversion.

Run the focused tests first, then `./gradlew.bat testClasses`; report any unrelated baseline failures separately.
