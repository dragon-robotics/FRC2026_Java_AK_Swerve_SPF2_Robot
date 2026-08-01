# Superstructure Full-Port Design

Date: 2026-07-31
Target branch: `feature/superstructure`
Reference: `FRC2026_Java_Swerve_Robot` `spitfire-v2` at
`70cce7cc0ee2c3b53644a582acf5b6bb9be35dd8`

## Objective

Port the complete `spitfire-v2` superstructure behavior into the AdvantageKit robot so the
drivetrain, intake, hopper, shooter, and vision subsystems operate as one driver-facing system.
The port includes the current driver/operator controls, PathPlanner named commands, Hub Shift
support, AdvantageKit telemetry, and deterministic tests. It adapts the reference behavior to the
already-approved target subsystem APIs instead of replacing those implementations.

The robot must be safe to enable during initial mechanical bring-up. Before the driver explicitly
enters `DRIVE`, the intake remains homed, the hopper remains stopped, and the shooter remains in its
low-speed preparation state.

## Scope

### In scope

- Replace the placeholder `Superstructure` with an AdvantageKit-native coordinator.
- Compose the existing `Drive`, `Intake`, `Hopper`, `Shooter`, and `Vision` subsystems.
- Port all reference superstates and manual shoot modes.
- Port alliance-aware zone selection, aim-point selection, alignment gating, and distance-based
  shooter setpoints.
- Port stationary target aiming using the target repository's `DriveCommands` architecture.
- Port delayed autonomous Juicer behavior and purge behavior.
- Port `HubShiftUtil`, its robot-mode lifecycle hooks, dashboard override, and telemetry.
- Port the current `spitfire-v2` driver and operator controller bindings, with the approved safe
  startup adaptation.
- Register the `Intake`, `Shoot`, `ShootNoAim`, and `Drive` PathPlanner named commands before the
  autonomous chooser is built.
- Add focused unit, command-requirement, binding, lifecycle, and simulation tests.

### Out of scope

- Motor IDs, current limits, PID gains, neutral modes, status frequencies, or other mechanism IO
  configuration.
- The approved Intake, Hopper, Shooter, Drive, or Vision state-machine internals.
- PathPlanner path files or autonomous routine geometry.
- Vision filtering, consensus, initialization, or reseeding algorithms.
- Hub Shift gating of shot commands. Hub Shift remains informational, matching the reference.
- Shoot-on-the-move behavior. The initial right-trigger aimed shot is stationary.

## Architecture and ownership

`Superstructure` extends `SubsystemBase` and receives `Drive`, `Intake`, `Hopper`, `Shooter`, and
`Vision` through its constructor. It does not hold a `RobotContainer` reference and does not create
hardware or simulation IO.

The class owns:

- the selected `Superstate`;
- the selected `ShootMode`;
- alliance confirmation and current `FieldZones` classification;
- the active alliance/zone-specific aim target;
- geometric heading alignment;
- distance-based shooter setpoint selection;
- heading-hold state shared with the default drive command;
- command factories that coordinate multiple subsystems; and
- superstructure and Hub Shift telemetry.

The class does not directly command mechanism outputs from `periodic()`. Cross-subsystem actuator
changes occur only inside WPILib commands with explicit requirements. Individual subsystem
state machines remain responsible for translating desired states into motor requests.

`RobotContainer` constructs Drive and Vision in their existing required order, constructs the three
mechanisms, constructs Superstructure, creates and registers named commands, builds the autonomous
chooser, and finally installs controller bindings.

## Public state model

Retain the target placeholder's `Superstate` name and values:

- `DRIVE_STARTING_CONFIG`
- `DRIVE`
- `INTAKE`
- `OUTTAKE`
- `SHOOT_WITH_AIM`
- `SHOOT_NO_AIM`
- `MANUAL_SHOOT`
- `PURGE`

Add `ShootMode`:

- `DEFAULT_SHOOT_WITH_AIM`
- `MANUAL_BUMPER_UP`
- `MANUAL_TRENCH`

Null constructor dependencies, requested states, and requested shoot modes are rejected
immediately. `ShootMode` persists until toggled again or the robot process restarts, matching the
reference behavior.

## State-command contract

| Superstate | Intake request | Hopper request | Shooter request | Drive ownership |
| --- | --- | --- | --- | --- |
| `DRIVE_STARTING_CONFIG` | `HOME` | `STOP` | `PREPFUEL` | Default drive remains available |
| `DRIVE` | `DEPLOYED` | `STOP` | `PREPFUEL` | Default drive remains available |
| `INTAKE` | `INTAKE` | `STOP` | `PREPFUEL` | Default drive remains available |
| `OUTTAKE` | `OUTTAKE` | `INDEXTOINTAKE` | `PREPFUEL` | Default drive remains available |
| `SHOOT_WITH_AIM` | Unchanged | Readiness-gated feed | `SHOOT` | Stationary target aim |
| `SHOOT_NO_AIM` | Unchanged | Readiness-gated feed | `SHOOT` | Unclaimed |
| `MANUAL_SHOOT` | Unchanged | Shooter-readiness-gated feed | `SHOOT` with fixed setpoint | Brake |
| `PURGE` | `OUTTAKE` | Readiness-gated feed | `SHOOT` | Stationary target aim |

State commands run until interrupted so press/release bindings and PathPlanner marker commands can
hand ownership from one state to the next. Commands require Superstructure plus every mechanism
they actively control. Aimed commands require Drive; `SHOOT_NO_AIM` does not. The independent
Juicer override requires only Intake so it may coexist with a non-purge driver shot.

Default mechanism commands provide the approved startup state:

- Intake: `HOME`
- Hopper: `STOP`
- Shooter: `PREPFUEL`

The logged initial superstate is `DRIVE_STARTING_CONFIG`, matching actual mechanism requests.

## Shooting and feed gating

Shooter readiness is `shooter.getCurrentState() == ShooterState.SHOOT`. This is intentionally used
instead of raw flywheel readiness because Shooter reaches `SHOOT` only after both its asymmetric
flywheel window and hood-position tolerance are satisfied.

For aimed, no-aim, and purge shots, Hopper changes to `INDEXTOSHOOTER` only when:

1. Shooter is in `SHOOT`; and
2. the robot's geometric field heading is strictly within 5 degrees of the active target.

`SHOOT_NO_AIM` retains the alignment gate but does not command or require Drive. Autonomous paths
must supply an aligned ending heading. Manual shooting omits the alignment condition and feeds once
Shooter is ready.

The default aimed shot is allowed only in:

- alliance left/right zones;
- alliance left/right trench zones;
- neutral left/right shoot zones; and
- neutral left/right purge zones.

It is disallowed in bump zones, plain neutral zones, and opponent zones. A disallowed default shot
does not claim Drive, alter a mechanism request, or interrupt normal driving. Zone and shoot-mode
decisions are evaluated when the trigger schedules the deferred command, not when bindings are
constructed.

Purge is selected automatically for the default shoot mode in neutral purge zones. It outtakes the
Intake, aims at the purge point, spins Shooter, and applies the same shooter-plus-alignment feed
gate.

## Shooter targets

Outside `MANUAL_SHOOT`, `periodic()` computes distance from the current Drive pose to the active
target and calls `Shooter.setSetpointForDistance(distanceMeters)` every loop.

Neutral shoot and purge zones retain the interpolated flywheel RPM but override the hood target to
the approved safe bring-up limit of **1.25 motor rotations**. This intentionally replaces the
reference's 2.0-rotation lock because the approved target calibration table currently ends at
1.25 rotations.

Manual modes use these approved initial constants:

| Mode | Flywheel | Hood |
| --- | ---: | ---: |
| `MANUAL_BUMPER_UP` | 2500 RPM | 0.0 rotations |
| `MANUAL_TRENCH` | 2900 RPM | 0.75 rotations |

## Field targeting and drivetrain aiming

When Driver Station alliance is unavailable, telemetry falls back to the blue hub but
`allianceConfirmed` remains false and zone-gated automatic shooting remains disabled. Once an
alliance is available, the current pose is classified with `FieldZones.fromPose()`.

Neutral shoot and purge zones use the existing alliance-specific `FieldConstants.AimPoints`.
Other allowed zones aim at the alliance hub center. Heading is calculated directly in the WPILib
field frame with `atan2(targetY - robotY, targetX - robotX)`.

The target repository's `DriveCommands.joystickDriveAtAngle()` supplies closed-loop aiming. The
stationary shot supplies zero translation and strafe. The raw geometric heading is passed for both
alliances; the CTRE reference's additional red-alliance pi rotation is not copied because the
AdvantageKit Drive command already handles red-alliance translation perspective separately.

The existing default-drive `currentDriveHeading` and `rotationLastTriggered` state moves from
`RobotContainer` into Superstructure. Stationary aiming refreshes the stored heading so returning
to default drive cannot recapture a stale pre-shot heading.

## Autonomous behavior

Register these commands before `AutoBuilder.buildAutoChooser()`:

- `Intake` -> persistent `INTAKE`
- `Shoot` -> aimed shoot with delayed Juicer behavior
- `ShootNoAim` -> no-aim shoot with delayed Juicer behavior
- `Drive` -> persistent `DRIVE`

The two autonomous shoot commands restart a private timer when scheduled. For the first 1.5
seconds, Intake remains `DEPLOYED`; afterward it changes to `JUICER`. The aimed variant owns Drive.
The no-aim variant does not. If the aimed command is in a purge zone, Intake uses `OUTTAKE` instead
of the Juicer sequence.

## Driver and operator controls

| Control | Command behavior |
| --- | --- |
| Driver sticks | Existing default drive with heading hold |
| Driver POV up | Existing half-speed behavior |
| Driver X | Existing zone-heading lock |
| Driver Start + Back | Reset field heading to zero while preserving translation |
| Driver left trigger above 0.2 | `INTAKE`; release -> `DRIVE` |
| Driver right bumper | `OUTTAKE`; release -> `DRIVE` |
| Driver right trigger above 0.2 | Deferred purge/default/manual shot; release -> `DRIVE` |
| Driver B | Intake `JUICER` while held; release -> `DEPLOYED` |
| Driver A | `DRIVE_STARTING_CONFIG` while held; release -> `DRIVE` |
| Operator Start + Back | Force Vision reseed |
| Operator B while A is not held | Intake `JUICER` while held; release -> `DEPLOYED` |
| Operator X + A | Toggle `MANUAL_BUMPER_UP` |
| Operator A + B | Toggle `MANUAL_TRENCH` |
| Operator right bumper | Kicker at 100% duty while held; normal Shooter state control resumes on release |

The right-trigger default shot is stationary. Manual shots brake Drive. All release paths explicitly
return the controlled mechanisms to a defined state; they do not depend on a command silently
ending with its last motor request active.

## Hub Shift support

Port `HubShiftUtil` without adding Lombok or another dependency. Preserve:

- official windows of 0-10, 10-35, 35-60, 60-85, 85-110, and 110-140 seconds;
- autonomous always active and disabled always inactive;
- the FMS match-time resynchronization threshold;
- game-specific-message/alliance winner selection;
- `HubShift/WonAuto` case-insensitive `true`/`false` override;
- shifted timing that opens an approaching active window 2.0 seconds early and closes an ending
  active window 0.5 seconds early; and
- public active/remaining-time accessors.

Call `HubShiftUtil.initialize()` from both `Robot.autonomousInit()` and `Robot.teleopInit()`.
Hub Shift data is informational and never suppresses a requested shot.

## Vision interaction

Superstructure calls `vision.setAiming(true)` only in `SHOOT_WITH_AIM` and `SHOOT_NO_AIM`, matching
the reference vision-noise behavior. It sets aiming false for drive, intake, outtake, manual shoot,
purge, and starting configuration. Operator force-reseed continues to call the existing Vision API
and keeps its current command requirement.

## AdvantageKit logging

Use `Logger.recordOutput()` rather than DogLog. At minimum publish:

- `Superstructure/CurrentState`
- `Superstructure/ShootMode`
- `Superstructure/AllianceConfirmed`
- `Superstructure/Alliance`
- `Superstructure/Zone`
- `Superstructure/AimTarget`
- `Superstructure/DistanceToTargetMeters`
- `Superstructure/DistanceToTargetFeet`
- `Superstructure/IsAlignedToTarget`
- `Superstructure/ShooterReady`
- `Superstructure/FeedReady`
- `Superstructure/ShootAllowed`
- `Superstructure/PurgeZone`
- `HubShift/Official/CurrentShift`
- `HubShift/Official/Active`
- `HubShift/Official/ElapsedTime`
- `HubShift/Official/RemainingTime`
- `HubShift/Shifted/CurrentShift`
- `HubShift/Shifted/Active`
- `HubShift/Shifted/ElapsedTime`
- `HubShift/Shifted/RemainingTime`
- `HubShift/FirstActiveAlliance`

Logging must clear or publish explicit fallback values when alliance/zone information is unavailable
so replay does not retain stale values.

## Error handling and safety

- Reject null dependencies, states, and modes.
- Never expose a desired transition-only mechanism state.
- Do not feed until Shooter has completed its own readiness transition.
- Do not feed automatic shots until geometric alignment is valid.
- Do not schedule a drive-owning idle command for a disallowed shot.
- Preserve normal drive when a default shot is disallowed.
- Keep startup Intake homed until driver A is released or another explicit state transition occurs.
- Restore `DRIVE` on the release paths for intake, outtake, shoot, and starting configuration.
- Restore normal Shooter control after the operator kicker override ends.
- Keep all target headings in the correct field-coordinate convention.

## Verification strategy

Implementation follows test-driven development. Focused tests precede each behavior change.

### Superstructure unit and command tests

- Initial state and actual default mechanism requests are safe and consistent.
- Every superstate requests the exact Intake, Hopper, and Shooter states in the contract table.
- Commands contain the exact required subsystems; `SHOOT_NO_AIM` does not require Drive.
- Aim-target selection is correct for blue/red hubs and neutral left/right shoot/purge points.
- Alignment uses a strict 5-degree boundary and correct angle wrapping.
- Zone restrictions include every allowed and disallowed zone.
- A disallowed shot neither commands mechanisms nor claims Drive.
- Shooter state plus alignment gates Hopper feeding.
- Manual shooting omits alignment gating and applies each approved preset.
- Neutral-zone hood target is exactly 1.25 rotations while preserving interpolated RPM.
- Vision aiming is true only for the two specified shoot states.
- Deferred commands observe the current zone and shoot mode at schedule time.
- Delayed autonomous commands switch to Juicer at the inclusive 1.5-second boundary.
- Purge substitutes Intake outtake for the Juicer sequence.

### Hub Shift tests

- Official shift boundaries, combined active windows, and remaining times.
- Shifted early-open and early-close boundaries.
- Blue/red alliance and game-specific-message winner selection.
- Dashboard override behavior.
- Autonomous, disabled, FMS resynchronization, and lifecycle reset behavior.

### RobotContainer and integration tests

- Drive/Vision construction order remains unchanged.
- All five subsystems and Superstructure are created in REAL, SIM, and REPLAY modes.
- Named commands are registered before the auto chooser is built.
- Controller ports and every press/hold/release binding match the approved table.
- Driver A release transitions from safe starting configuration to `DRIVE`.
- Right-trigger purge/default/manual routing is evaluated at schedule time.
- Existing heading reset and vision reseed bindings remain functional.
- Physics simulation demonstrates intake, hopper, shooter, and drive behavior under coordinated
  commands.

### Final gates

- Focused Superstructure, Hub Shift, RobotContainer, and command tests.
- Full `gradlew check` with isolated Vision simulation tests.
- Spotless and `git diff --check`.
- Independent review of the final branch diff against this specification.

## Expected file impact

Primary files expected to change or be added:

- `src/main/java/frc/robot/subsystems/Superstructure.java`
- `src/main/java/frc/robot/util/HubShiftUtil.java`
- `src/main/java/frc/robot/RobotContainer.java`
- `src/main/java/frc/robot/Robot.java`
- `src/main/java/frc/robot/commands/DriveCommands.java` only if a small aiming/heading-state API
  adaptation is required
- focused tests under `src/test/java/frc/robot/`

Generated AdvantageKit input classes and the approved mechanism IO/configuration files are not
manually edited.

## Acceptance criteria

The port is complete when:

1. all approved superstates, shooting modes, zone rules, target calculations, bindings, named
   commands, and Hub Shift behavior are implemented;
2. startup and every release path leave mechanisms in an explicit safe state;
3. automatic feeding requires Shooter readiness and the approved alignment condition;
4. the current AdvantageKit mechanism and drive ownership boundaries remain intact;
5. AdvantageKit logs expose the complete coordination state without stale values;
6. the focused and full verification suites pass; and
7. the final diff contains no unrelated mechanism tuning or configuration changes.
