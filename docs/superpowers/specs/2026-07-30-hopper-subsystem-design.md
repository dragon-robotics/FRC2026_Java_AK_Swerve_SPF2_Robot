# Hopper Subsystem Design

## Goal

Add the complete hopper foundation needed before a future Superstructure coordinates it with the
intake and shooter. The port preserves the `spitfire-v2` hopper hardware values and behaviors while
adapting the implementation to this repository's AdvantageKit IO architecture.

## Scope

Included:

- Hopper state machine with stop, index-to-shooter, and index-to-intake states.
- Direct voltage, duty-cycle percentage, and torque-current FOC control methods.
- Real TalonFX IO for a lead/follower roller pair.
- Physics simulation IO for both roller motors.
- Replay-safe no-op IO.
- AdvantageKit input logging for both motors and state output logging.
- Real, simulation, and replay construction in `RobotContainer`.
- Focused unit tests for subsystem behavior.

Excluded:

- Controller bindings.
- Intake or shooter behavior.
- Superstructure coordination.
- Autonomous named commands.
- Fuel sensing or jam detection not present in the reference.

## Architecture

The hopper will use a dedicated AdvantageKit IO boundary rather than copying the source
repository's generic `MotorIO` framework. `HopperIO` exposes telemetry for both motors and the three
control modes used by `Hopper`: voltage, duty-cycle percentage, and torque-current FOC.

`HopperIOTalonFX` owns and configures both TalonFX controllers. The lead motor receives commands and
the second motor follows it. `HopperIOSim` models both motors and retains the commanded control mode
between periodic updates. Replay mode constructs `Hopper` with the no-op `HopperIO` implementation.

`Hopper` owns all state semantics. Callers set a desired state; during `periodic()`, the subsystem
updates logged inputs and sends a motor command only when the desired state differs from the last
commanded state. The initial periodic call sends the STOP command. This keeps CAN traffic low while
ensuring startup behavior is explicit.

## Components

### `HopperConstants`

Preserve these `spitfire-v2` values:

- Lead TalonFX CAN ID: 17.
- Follower TalonFX CAN ID: 18.
- Stator current limit: 80 A.
- Supply current limit: 40 A.
- Supply lower limit: 20 A after 0.2 seconds.
- Maximum voltage: 12 V.
- Index to shooter: +12 V.
- Index to intake: -12 V.
- Stop: 0 V.
- Open-loop ramp period: 0.5 seconds.
- Torque-current FOC maximum absolute duty cycle: 1.0 (100%).
- Lead inversion: counterclockwise-positive.
- Follower inversion: clockwise-positive.
- Neutral mode: coast.
- Real follower relationship: aligned with the leader command, matching the source competition
  construction.

Simulation-only motor inertia and gearing will live beside the simulation implementation because
the source repository does not define mechanism measurements for them.

### `HopperIO`

The `@AutoLog` input snapshot records, separately for lead and follower:

- Connection status.
- Position in rotations.
- Velocity in RPM.
- Applied voltage.
- Stator current in amps.
- Temperature in Celsius.

Control methods accept voltage, duty-cycle percentage, or torque current. Implementations clamp
percentage to the valid `[-1.0, 1.0]` range, voltage to the configured maximum, and torque current
to the configured stator current limit.

### `HopperIOTalonFX`

The real implementation applies the preserved current, voltage, ramp, neutral-mode, and inversion
settings. It retries TalonFX configuration through the repository's existing `PhoenixUtil`
mechanism, configures the second controller as a hardware follower, sends torque commands with
Phoenix 6 `TorqueCurrentFOC.withMaxAbsDutyCycle(1.0)`, reads both controllers' telemetry, debounces
communication loss, and reduces unused CAN traffic.

### `HopperIOSim`

The simulation implementation uses two `DCMotorSim` models driven by the same requested hopper
command. Voltage and percentage commands are open-loop. Torque-current commands are converted
through the motor model into equivalent simulated torque. Each update advances both models by 20 ms
and publishes the same input fields as real hardware.

### `Hopper`

The public state enum contains:

- `STOP`
- `INDEX_TO_SHOOTER`
- `INDEX_TO_INTAKE`

`setDesiredState()` rejects null and stores the requested state. `getDesiredState()` returns that
request. `getCurrentState()` returns the most recently commanded state.

On state entry:

- STOP commands 0 V.
- INDEX_TO_SHOOTER commands +12 V.
- INDEX_TO_INTAKE commands -12 V.

Direct control methods forward voltage, percentage, and torque-current FOC requests to IO. State
transitions remain entry-based, so a direct request remains active until a different state is
requested.

`periodic()` updates and logs inputs before handling a state transition. It also logs current and
desired state names through AdvantageKit outputs.

### `RobotContainer`

Construction follows existing mode selection:

- REAL: `HopperIOTalonFX`.
- SIM: `HopperIOSim`.
- REPLAY: no-op `HopperIO`.

The constructed hopper is retained as a subsystem field for later Superstructure integration. No
default command or controller binding is added.

## Error Handling

- Null desired states fail immediately with `NullPointerException` through `Objects.requireNonNull`.
- Hardware configuration uses bounded retries and reports failures through existing Phoenix status
  handling.
- Connection flags use falling-edge debouncing so one missed status frame does not create noisy
  disconnect logs.
- Output requests are clamped before reaching hardware or simulation.

## Testing

Focused tests use a recording fake `HopperIO` and verify:

- First periodic call sends STOP.
- Each state sends the correct configured voltage.
- Repeated periodic calls in the same state do not resend commands.
- Desired state changes only become current after periodic processing.
- Voltage, percentage, and torque-current FOC direct controls forward correctly.
- Torque-current FOC requests use a maximum absolute duty cycle of 1.0.
- Null desired states are rejected.

Simulation tests verify output clamping and that commanded voltage and torque current produce motor
motion. Full verification runs focused hopper tests, all project tests, compilation, Spotless
checks, and `git diff --check`.
