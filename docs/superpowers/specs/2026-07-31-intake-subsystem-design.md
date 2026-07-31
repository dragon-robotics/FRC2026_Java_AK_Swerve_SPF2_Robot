# Intake Subsystem Design

## Goal

Port the complete intake foundation from the `spitfire-v2` reference into this repository's
AdvantageKit architecture before Superstructure coordination exists. Preserve the tuned intake,
outtake, deploy, stow, and juicer behavior while adapting the arm for its new 40:1 geared final
pivot and reducing collision loads on the gear teeth.

The reference baseline is commit `70cce7cc0ee2c3b53644a582acf5b6bb9be35dd8` from
`FRC2026_Java_Swerve_Robot:spitfire-v2`.

## Scope

Included:

- One intake subsystem controlling two opposed rollers and one deployable arm.
- TalonFX hardware IO for all three motors and fused CANcoder arm feedback.
- Full dynamic simulation for the rollers and gravity-loaded arm.
- Replay-safe no-op IO.
- AdvantageKit input and state logging.
- Real, simulation, and replay construction in `RobotContainer`.
- Focused state, hardware-configuration, simulation, logging, and mode-construction tests.

Excluded:

- Controller bindings and default commands.
- Superstructure implementation or coordination.
- Autonomous named commands.
- Fuel sensing, jam detection, or collision detection.
- Roller RPM control.
- Automatic arm redeployment after a collision while a brake-neutral steady state remains active.
- Unused Motion Magic configuration.

## Architecture

The intake uses one subsystem-specific AdvantageKit IO boundary, matching the existing hopper and
shooter architecture.

- `IntakeConstants` owns IDs, limits, setpoints, gains, request timing, configuration factories,
  and simulation parameters.
- `IntakeIO` defines one logged telemetry snapshot and typed commands for the rollers, arm, and
  CANcoder.
- `IntakeIOTalonFX` owns and configures both roller TalonFX controllers, the arm TalonFX, and the
  arm CANcoder.
- `IntakeIOSim` models two rollers and the full arm mechanism.
- `Intake` owns desired/current state, transition logic, juicer phases, readiness, direct controls,
  and AdvantageKit outputs.
- `RobotContainer` retains the subsystem and selects real, simulation, or replay IO. It adds no
  behavior bindings.

This avoids the reference repository's generic `MotorIO` capability checks. Every supported intake
command is explicit and testable at one mechanism boundary.

## Hardware and Configuration

### Rollers

- Lead TalonFX: CAN ID 21.
- Follower TalonFX: CAN ID 20.
- Follower alignment: opposed to the lead.
- Lead inversion: clockwise-positive.
- Neutral mode: coast.
- Stator current limit: 80 A.
- Supply current limit: 40 A.
- Supply lower limit: 30 A after 0.2 seconds.
- Peak voltage: +12 V / -12 V.
- Intake torque request: +80 A with 0.80 maximum absolute duty cycle.
- Outtake torque request: -80 A with 0.80 maximum absolute duty cycle.
- Juicer torque request: +80 A with 0.50 maximum absolute duty cycle.

The rollers expose voltage, duty-cycle percentage, and torque-current FOC controls. RPM control is
intentionally omitted because no intake state uses it.

### Arm and CANcoder

- Arm TalonFX: CAN ID 10.
- Arm CANcoder: CAN ID 0.
- Motor inversion: clockwise-positive.
- Motor neutral mode: brake.
- CANcoder direction: clockwise-positive.
- CANcoder magnet offset: `0.881064453125` rotations.
- Absolute-sensor discontinuity point: `0.5` rotations.
- Feedback source: fused CANcoder.
- Sensor-to-mechanism ratio: 1:1.
- Rotor-to-sensor ratio: 40:1, updated from the reference's 36:1 ratio for the geared final pivot.
- Stator current limit: 50 A.
- Supply current limit: 30 A.
- Peak voltage: +10 V / -10 V.

Closed-loop slots:

- Slot 0, fast deploy and pre-juice: `kP = 14.0`.
- Slot 1, slow stow and juicer squeeze: `kP = 8.0`.
- Both slots: `kV = 2.4`, `kG = 0.5`, all other gains zero.
- Gravity type: arm cosine.
- Static feedforward sign: use the closed-loop sign.

Arm positions use fused mechanism rotations:

- Deployed: `0.00` rotations.
- Juicer pre-position: `0.15` rotations.
- Juicer squeeze: `0.25` rotations.
- Stowed: `0.37` rotations.
- Position readiness tolerance: inclusive within `0.025` rotations.

`PositionVoltage` performs arm position control. Motion Magic values are not configured because
Phoenix does not apply them to `PositionVoltage` requests.

### Control Requests and CAN Timing

- All control requests update at 100 Hz.
- `VoltageOut`, `DutyCycleOut`, and `PositionVoltage` enable FOC.
- `PositionVoltage` selects slot 0 or slot 1 per command.
- Direct `TorqueCurrentFOC` requests use a 1 A deadband and override coast during neutral.
- Roller torque requests select a 0.80 or 0.50 maximum duty cycle per state.
- Arm torque requests use a 1.0 maximum absolute duty cycle.
- `NeutralOut` uses the arm motor's configured brake neutral mode. Phoenix `NeutralOut` has no
  brake-override field.
- The opposed follower request updates at 100 Hz.
- Position, velocity, applied-voltage, and stator-current signals update at 50 Hz.
- Temperature signals update at 4 Hz.
- CANcoder absolute position, position, and velocity update at 50 Hz.
- Unspecified signals update at 4 Hz after bus-utilization optimization.

All voltage, percentage, torque-current, position-slot, and torque-duty values are clamped or
validated at the subsystem boundary and again in the IO implementation.

## IO Contract

`IntakeIOInputs` records:

- Roller lead connection, position rotations, velocity RPM, applied volts, stator current amps,
  and temperature Celsius.
- Roller follower connection, position rotations, velocity RPM, applied volts, stator current
  amps, and temperature Celsius.
- Arm TalonFX connection, fused mechanism position rotations, mechanism velocity RPM, applied
  volts, stator current amps, and temperature Celsius.
- CANcoder connection, configured position rotations, absolute position rotations, and velocity
  RPM.

`IntakeIO` exposes:

- Roller voltage, duty cycle, and torque current with a maximum absolute duty-cycle argument.
- Arm `PositionVoltage` setpoint with slot selection.
- Arm voltage, duty cycle, and torque current.
- Explicit arm brake-neutral output.

The follower remains hardware-controlled and has no independent output method.

## State Machine

Public steady states:

- `HOME`
- `INTAKE`
- `OUTTAKE`
- `DEPLOYED`
- `JUICER`

Internal transition states:

- `DEPLOYING`
- `STOWING`

`setDesiredState()` rejects null and rejects `DEPLOYING` or `STOWING` as desired states. A new
steady-state request selects the required transition based on measured arm position, not only the
previous state. This allows a later state change to redeploy an arm displaced by a collision.
Repeated requests for the active state do not resend CAN commands.

Commands are sent only on state or juicer-phase entry. Inputs and logs update every loop. When a
transition reaches its tolerance, the new steady outputs are applied before `currentState` changes,
so state reporting always matches the commanded outputs in the same periodic cycle.

### Startup and HOME

Startup sets desired state to `HOME` and current state to `STOWING`. The first periodic cycle
explicitly stops the rollers and commands the arm to `0.37` rotations with slow slot 1. When the arm
is within tolerance, current state becomes `HOME`. The stowed position request remains active in
HOME.

### DEPLOYING

The arm receives a fast-slot position request to `0.00` rotations. During deployment:

- Autonomous runs the roller outward at -6 V.
- Teleop, disabled simulation, and other modes command roller 0 V.

When the arm reaches deployed tolerance, the subsystem immediately applies outputs for the desired
steady state:

- `DEPLOYED`: brake neutral and roller stop.
- `INTAKE`: -15 A arm tension and +80 A roller torque at 0.80 maximum duty.
- `OUTTAKE`: brake neutral and -80 A roller torque at 0.80 maximum duty.

Only then does `currentState` change.

### DEPLOYED

The arm receives explicit `NeutralOut` under its configured brake mode, and the rollers receive
0 V. No position PID or torque hold remains active. A collision may backdrive the arm rather than
causing the position controller to fight through the final gear teeth. The state machine does not
automatically re-enable position control if the arm moves away from deployed while this state
remains active.

### INTAKE

The roller receives +80 A torque current with a 0.80 maximum duty cycle. The arm receives -15 A
torque current to keep the deployed mechanism rigid. This is the only deployed steady state that
accepts the collision-load risk of active arm tension.

### OUTTAKE

The roller receives -80 A torque current with a 0.80 maximum duty cycle. The arm remains in explicit
brake neutral with no position PID or torque hold.

### JUICER

JUICER restarts from `PRE_JUICE` on every entry:

1. The roller receives +80 A torque current with a 0.50 maximum duty cycle.
2. `PRE_JUICE` commands `0.15` rotations with fast slot 0.
3. At inclusive tolerance, `SQUEEZE` commands `0.25` rotations with slow slot 1.

The subsystem contains no timer. Any future delay before entering JUICER belongs to Superstructure.

### STOWING

The rollers receive 0 V. The arm receives a slow-slot position request to `0.37` rotations. At
inclusive tolerance, current state becomes `HOME` while the same stowed position request remains
active.

## Direct Controls

Public direct methods expose all IO-supported roller and arm controls. They clamp requests before
forwarding. Normal periodic state behavior remains entry-driven, so a future manual command must
require the intake subsystem, choose its control path deliberately, and command a safe terminal
state when ending.

## AdvantageKit Logging

Every periodic cycle:

1. Refreshes all TalonFX and CANcoder signals.
2. Populates every input field.
3. Calls `Logger.processInputs("Intake", inputs)`.
4. Advances the state machine.
5. Records desired state, current state, juicer phase, measured arm position, deployed readiness,
   stowed readiness, and pre-juice readiness as AdvantageKit outputs.

Temperatures are explicitly refreshed in hardware and explicitly set to `0.0` in simulation.

## Simulation

`IntakeIOSim` uses:

- Two Kraken X60 FOC roller models with opposed follower motion.
- One Kraken X60 FOC `SingleJointedArmSim`.
- 40:1 arm gearing.
- 13.370-inch arm length.
- 10 lb arm mass.
- Physical limits from `0.00` through `0.37` rotations.
- Initial arm position of `0.37` rotations.
- Gravity and hard stops.
- A 20 ms update period.

Retained simulation modes cover roller voltage, duty cycle, and torque current; arm position,
voltage, duty cycle, torque current, and brake neutral. Position control uses the selected slot's
gains. Torque current converts through the motor model and is bounded by configured voltage and
current limits. Brake neutral applies zero commanded voltage and allows the motor model's electrical
braking and mechanism physics to determine motion.

Simulation calculates combined mechanism current draw and applies battery-voltage sag before final
motor inputs. It publishes configured/fused CANcoder values from simulated mechanism position,
reports all devices connected, and overwrites every temperature field with `0.0` Celsius.

## RobotContainer Integration

`RobotContainer` retains a private `Intake` field and constructs mode-specific IO:

- REAL: `IntakeIOTalonFX`.
- SIM: `IntakeIOSim`.
- REPLAY: `IntakeIO.NoOp`.

The REAL factory remains testable without constructing CAN hardware, following the shooter factory
pattern. No controller binding, default command, named command, or Superstructure logic is added.

## Error Handling

- Null desired states fail immediately.
- Internal transition states cannot be requested externally.
- Hardware configuration, sticky-fault clearing, and follower setup use bounded retries.
- Every read status signal is refreshed before access.
- Each device has a falling-edge connection debouncer.
- Outputs are clamped at subsystem and IO boundaries.
- Replay performs no hardware or simulation IO.
- Brake-neutral states deliberately do not auto-redeploy after collision displacement.

## Testing

Subsystem tests verify:

- Safe first-periodic startup.
- Every desired-state and internal transition path.
- Same-cycle alignment between current state and commanded outputs.
- DEPLOYED and OUTTAKE use brake neutral and never leave position PID active.
- INTAKE uses -15 A arm tension.
- Autonomous deployment uses -6 V roller output; teleop deployment uses 0 V.
- Roller intake/outtake use the 0.80 duty cap; JUICER uses 0.50.
- Juicer phase sequencing and fast/slow position slots.
- Inclusive `0.025` readiness boundaries.
- Null and internal-state request rejection.
- Direct-control clamping.

Phoenix configuration tests verify without constructing hardware:

- CAN IDs, inversions, neutral modes, and opposed follower alignment.
- Roller and arm current/voltage limits.
- CANcoder ID 0, direction, offset, and discontinuity.
- Fused feedback and the 40:1 rotor-to-sensor ratio.
- Slot gains and gravity settings.
- Request types, FOC flags, deadband, maximum duty cycles, neutral output, and 100 Hz control timing.
- 50 Hz mechanism/CANcoder signals plus 4 Hz temperature and unspecified signals.
- Temperature and CANcoder signals belong to explicit refresh groups.

Simulation tests verify:

- Every input field is overwritten.
- Both rollers move with opposed telemetry under all direct modes.
- Arm position control reaches deploy, juicer, and stow setpoints.
- The 0.00 and 0.37 hard limits are enforced.
- Brake neutral does not continue position PID.
- Torque-current tension produces bounded arm response.
- Battery voltage clamps applied output under combined load.
- Simulated CANcoder tracks the arm.

`RobotContainer` tests verify REAL mapping without hardware construction, full SIM IO, and no-op
REPLAY IO. Final verification runs all tests, compilation, Spotless, and `git diff --check`.
