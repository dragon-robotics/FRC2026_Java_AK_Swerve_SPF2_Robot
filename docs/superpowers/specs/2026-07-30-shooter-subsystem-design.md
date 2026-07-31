# Shooter Subsystem Design

## Goal

Add the complete shooter foundation needed before the future Superstructure coordinates scoring.
The subsystem will preserve the `spitfire-v2` shooter hardware, state model, tuned setpoints, and
distance interpolation while adapting the implementation to this repository's AdvantageKit IO
architecture.

## Scope

Included:

- One shooter subsystem controlling the flywheel pair, kicker, and adjustable hood.
- Flywheel closed-loop RPM control using Phoenix `VelocityTorqueCurrentFOC`.
- Hood closed-loop position control using Phoenix `PositionVoltage`.
- Direct voltage, duty-cycle percentage, and torque-current FOC control for the commanded motors.
- Real TalonFX IO for all four motors.
- Full physics simulation for the flywheels, kicker, and hood.
- Replay-safe no-op IO.
- AdvantageKit input and state logging.
- Real, simulation, and replay construction in `RobotContainer`.
- Focused tests for configuration, simulation, state behavior, and mode construction.

Excluded:

- Controller bindings.
- Superstructure coordination.
- Autonomous named commands.
- Vision aiming and distance calculation.
- Hood CANcoder hardware or configuration.
- Kicker clearing delay after STOP.
- Fuel sensing or jam detection.

## Architecture

The shooter will use one subsystem-specific AdvantageKit IO boundary. `ShooterIO` owns the
telemetry and commands for the flywheel lead and follower, kicker, and hood. This keeps the four
coordinated mechanisms in one log group and avoids importing the reference repository's generic
`MotorIO` abstraction.

`ShooterIOTalonFX` owns and configures all four TalonFX controllers. The flywheel lead receives
commands and the second flywheel motor follows it in the opposed direction. The kicker and hood
receive independent commands. `ShooterIOSim` models all four motors and supports every exposed
control mode. Replay mode constructs `Shooter` with a no-op `ShooterIO`.

`Shooter` owns the state machine, setpoint selection, readiness checks, and state-output logging.
`RobotContainer` retains the constructed shooter as a subsystem field but adds no default command
or operator binding before the Superstructure is implemented.

## Components

### `ShooterConstants`

Hardware mapping:

- Hood TalonFX: CAN ID 13.
- Kicker TalonFX: CAN ID 14.
- Flywheel lead TalonFX: CAN ID 15.
- Flywheel follower TalonFX: CAN ID 16.
- Flywheel follower alignment: opposed to the lead.
- No hood CANcoder.

Flywheel configuration:

- Neutral mode: coast.
- Lead inversion: clockwise-positive.
- Stator current limit: 100 A.
- Supply current limit: 40 A.
- Supply lower limit: 20 A after 0.25 seconds.
- Peak voltage: +12 V / -12 V.
- Peak torque current: +120 A / -40 A.
- Slot 0 gains: `kP = 8.0`, `kS = 4.325`, and `kV = 0.013`; all other gains are zero.
- Default shooting target: 2500 RPM.
- Prep target: 1200 RPM.
- Ready range: no more than 120 RPM below target and no more than 60 RPM above target.
- Stopped tolerance: 0.5 RPM.

Kicker configuration:

- Inversion: clockwise-positive.
- Neutral mode: coast.
- Stator current limit: 80 A.
- Supply current limit: 40 A.
- Supply lower limit: 20 A after 0.25 seconds.
- Peak voltage: +12 V / -12 V.
- Prep output: 6 V.
- Shooting output: 12 V.
- STOP output: 0 V with no delayed clearing run.

Hood configuration:

- Inversion: clockwise-positive.
- Neutral mode: brake.
- Integrated TalonFX encoder feedback.
- Startup position reset: 0.0 rotations, based on the requirement that the mechanism is
  mechanically parked at zero when the robot boots.
- Stator current limit: 30 A.
- Supply current limit: 15 A.
- Peak voltage: +10 V / -10 V.
- Slot 0 gains: `kP = 8.0`, `kD = 0.1`, and `kG = 0.4`; all other gains are zero.
- Gravity type: elevator static.
- Static feedforward sign: use the closed-loop sign.
- Default position: 0.0 rotations.
- Ready tolerance: 0.125 rotations.

Control and status timing:

- All control requests update at 100 Hz.
- Voltage and duty-cycle requests enable FOC.
- Torque-current requests use a 1 A deadband.
- Torque-current requests use a 1.0 maximum absolute duty cycle.
- Torque-current requests override coast during neutral.
- Position, velocity, applied-voltage, and stator-current signals update at 50 Hz.
- Temperature signals update at 4 Hz.
- Unspecified signals update at 4 Hz after CAN-bus optimization.

Distance interpolation preserves the reference table:

| Distance | Flywheel target | Hood target |
| ---: | ---: | ---: |
| 5 ft | 2400 RPM | 0.00 rotations |
| 6 ft | 2475 RPM | 0.00 rotations |
| 7 ft | 2525 RPM | 0.00 rotations |
| 8 ft | 2675 RPM | 0.00 rotations |
| 9 ft | 2750 RPM | 0.00 rotations |
| 10 ft | 2850 RPM | 0.75 rotations |
| 11 ft | 2900 RPM | 0.75 rotations |
| 12 ft | 3000 RPM | 1.25 rotations |

Authored distances remain in feet for readability and are converted to meters before insertion
into WPILib interpolating maps.

### `ShooterIO`

The `@AutoLog` input snapshot records the following separately for the flywheel lead, flywheel
follower, kicker, and hood:

- Connection status.
- Position in rotations.
- Velocity in RPM.
- Applied voltage.
- Stator current in amps.
- Temperature in Celsius.

The IO interface exposes:

- Flywheel velocity in RPM, voltage, duty-cycle percentage, and torque current.
- Kicker voltage, duty-cycle percentage, and torque current.
- Hood position in integrated-motor rotations, voltage, duty-cycle percentage, and torque current.
- Hood integrated-position reset.

Implementations clamp percentage to `[-1.0, 1.0]`, voltage to each mechanism's configured peak
voltage, and torque current to the effective configured range. For the flywheel, this means the
request cannot exceed the 100 A stator limit or the asymmetric -40 A reverse peak; kicker and hood
requests cannot exceed their stator-current limits. The flywheel follower remains
hardware-controlled and does not expose an independent output method.

### `ShooterIOTalonFX`

The real implementation uses:

- `VelocityTorqueCurrentFOC` for flywheel RPM.
- `PositionVoltage` for hood position.
- `VoltageOut` for direct voltage.
- `DutyCycleOut` for direct percentage.
- `TorqueCurrentFOC` for direct torque current.
- `Follower` for the opposed flywheel follower.

Flywheel velocity and direct torque-current requests use a 1 A deadband, 1.0 maximum absolute duty
cycle, coast-during-neutral override, and 100 Hz update frequency. Voltage and duty-cycle requests
enable FOC and update at 100 Hz. Hood position and follower requests also update at 100 Hz.

Each controller configuration is applied through the repository's bounded `PhoenixUtil` retry
helper, then sticky faults are cleared. Every status signal used by `updateInputs()` is refreshed
before its value is read. Each motor has a falling-edge connection debouncer so one missed status
frame does not produce noisy disconnect telemetry.

Required mechanism signals update at 50 Hz and temperatures at 4 Hz. After setting explicit signal
frequencies, the implementation calls `ParentDevice.optimizeBusUtilizationForAll(4.0, ...)` for all
four TalonFX controllers so unspecified signals also update at 4 Hz.

### `ShooterIOSim`

The simulation implementation maintains separate physical models for both flywheels, the kicker,
and the hood. Flywheel and kicker models use Kraken X60 FOC motor characteristics. The hood uses
Kraken X44 characteristics and a motor-position simulation because the reference repository does
not provide the geometry, mass, hard stops, or gearing needed for a reliable arm model.

Simulation retains a control mode for each commanded mechanism:

- Flywheel: voltage, duty cycle, torque current, or velocity.
- Kicker: voltage, duty cycle, or torque current.
- Hood: voltage, duty cycle, torque current, or position.

Velocity and position modes calculate bounded voltage commands that move their models toward the
requested setpoints. The opposed follower receives the corresponding follower command and publishes
independent changing telemetry. Each update advances every model by 20 ms and populates every
`ShooterIOInputs` field. Simulated controllers report connected, and temperature fields are
explicitly set to 0.0 degrees Celsius.

### `Shooter`

The public state enum contains:

- `STOP`
- `PREPFUEL`
- `SHOOT`
- `TRANSITION`

`setDesiredState()` rejects null. Requests for STOP, PREPFUEL, or SHOOT move the current state
through TRANSITION before the requested steady state. Startup explicitly runs the transition to
STOP so all four outputs receive safe commands on the first periodic update.

STOP behavior:

- Flywheel receives 0 V.
- Kicker immediately receives 0 V.
- Hood returns to 0.0 rotations.
- No kicker timer or clearing output exists.
- The current state becomes STOP when absolute flywheel speed is below 0.5 RPM.

PREPFUEL behavior:

- Flywheel receives a 1200 RPM velocity request.
- Kicker receives 6 V.
- Hood receives a 0.0-rotation position request.
- The current state becomes PREPFUEL when flywheel speed is from 1080 through 1260 RPM, inclusive.

SHOOT behavior:

- Flywheel receives the selected RPM request.
- Hood receives the selected position request.
- Kicker remains at the 6 V prep output during the transition.
- Flywheel readiness is inclusive from `targetRPM - 120` through `targetRPM + 60`.
- Hood readiness is inclusive within 0.125 rotations of its target.
- The current state becomes SHOOT only when both mechanisms are ready.
- In SHOOT, the kicker receives 12 V while the flywheel and hood continue receiving their selected
  closed-loop requests.

`setSetpointForDistance()` accepts meters and selects interpolated flywheel and hood targets.
`setSetpoint()` directly selects both targets. Accessors expose desired/current state, target RPM,
target hood position, measured flywheel speed, and readiness information.

Direct public methods forward voltage, percentage, and torque-current requests for the flywheel,
kicker, and hood. Closed-loop flywheel RPM and hood position methods are also available. The state
machine continues to own normal periodic behavior, so future manual commands must require the
subsystem and deliberately select the desired control path.

`periodic()` updates and logs `ShooterIOInputs`, advances the state machine, and records desired
state, current state, target RPM, target hood position, measured flywheel RPM, flywheel readiness,
and hood readiness through AdvantageKit outputs.

### `RobotContainer`

Construction follows the existing mode selection:

- REAL: `ShooterIOTalonFX`.
- SIM: `ShooterIOSim`.
- REPLAY: no-op `ShooterIO`.

The shooter is retained as a subsystem field for later Superstructure integration. No default
command or controller binding is added.

## Error Handling

- Null desired states fail immediately through `Objects.requireNonNull`.
- Hardware configuration and sticky-fault operations use bounded retries.
- Connection flags use falling-edge debouncing.
- Every read status signal, including temperature, is explicitly refreshed.
- Output requests are clamped before reaching hardware or simulation.
- Replay mode performs no hardware or simulation IO.

## Testing

Focused subsystem tests use a recording fake `ShooterIO` and verify:

- The first periodic call commands a safe STOP.
- STOP halts the kicker immediately and contains no delayed clearing behavior.
- PREPFUEL commands 1200 RPM, 6 V kicker output, and zero hood position.
- SHOOT does not feed at 12 V until both flywheel and hood are ready.
- Flywheel readiness accepts exactly 120 RPM below target and exactly 60 RPM above target.
- Flywheel readiness rejects values below and above those asymmetric boundaries.
- Hood readiness uses the 0.125-rotation tolerance.
- Desired states transition through TRANSITION before becoming current.
- Direct voltage, duty-cycle, and torque-current methods forward and clamp correctly.
- Distance-based and manual setpoint selection return the expected targets.

Phoenix configuration tests verify without constructing hardware:

- CAN IDs and follower opposition.
- Flywheel, kicker, and hood current and voltage limits.
- The 30 A hood stator-current limit.
- Flywheel velocity request type and settings.
- Hood position request type and settings.
- Voltage, duty-cycle, torque-current, and follower request settings.
- The 50 Hz mechanism, 4 Hz temperature, and 4 Hz unspecified-signal schedule.

Simulation tests verify:

- Every input field is explicitly populated.
- Voltage, duty-cycle, and torque-current commands produce changing telemetry.
- Flywheel velocity control approaches its RPM target.
- Hood position control approaches its rotation target.
- Kicker output changes dynamically.
- Follower telemetry changes with the lead.
- Outputs are clamped to configured limits.

`RobotContainer` tests verify real, simulation, and replay factory selection without constructing
real hardware in unit tests. Full verification runs focused shooter tests, all project tests,
compilation, Spotless checks, and `git diff --check`.
