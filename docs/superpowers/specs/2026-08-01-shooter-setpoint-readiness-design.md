# Shooter Setpoint Readiness Design

Date: 2026-08-01
Target branch: `feature/superstructure`

## Problem

`Shooter.periodic()` can transition the Shooter to `SHOOT` and log that state. Later in the same
command-scheduler loop, `Superstructure.periodic()` refreshes the distance-based Shooter setpoint.
`Shooter.setSetpoint()` currently changes the Shooter back to `TRANSITION` whenever either target
changes by any amount. The shoot command then observes `TRANSITION` at its Hopper feed check, so the
Hopper remains in `STOP` even though the measured flywheel speed and hood position satisfy the
refreshed target.

Small pose changes can trigger this behavior continuously. Neutral shoot and purge zones expose a
deterministic version because Superstructure first applies the interpolated hood target and then
overrides it with the 1.25-rotation neutral-zone limit every loop.

## Readiness Windows

Keep the existing Superstructure contract: Hopper feeding still requires
`shooter.getCurrentState() == ShooterState.SHOOT`. Do not bypass the Shooter state machine, change
the scheduler order, or gate Hopper directly from raw sensor values.

Shooter uses separate acquisition and maintenance windows:

| Phase | Flywheel lower bound | Flywheel upper bound | Hood tolerance |
| --- | ---: | ---: | ---: |
| Entering `SHOOT` from `TRANSITION` | Target - 60 RPM | Target + 60 RPM | Target +/- 0.125 rotations |
| Remaining in `SHOOT` | Target - 120 RPM | Target + 60 RPM | Target +/- 0.125 rotations |

All bounds are inclusive. The tighter symmetric acquisition window prevents feeding until the
flywheel has reached its target. The wider downward maintenance window accounts for the expected
speed drop as balls load the flywheel without unnecessarily stopping the shot. Exceeding either
maintenance bound, or leaving the hood tolerance, immediately returns Shooter to `TRANSITION` and
restores the 6 V kicker preparation output until the acquisition window is satisfied again.

`Shooter/FlywheelReady` reports readiness for the active phase: the acquisition window while in
`TRANSITION`, and the maintenance window while in `SHOOT`. Hood readiness uses the same 0.125-
rotation tolerance in both phases.

## Setpoint Refresh Behavior

When `Shooter.setSetpoint()` changes a target while Shooter is already in `SHOOT`, evaluate the
most recent measurements against the new target's maintenance window:

- If both measurements satisfy the maintenance tolerances, retain `SHOOT`.
- If either measurement is outside its maintenance tolerance, enter `TRANSITION` and require the
  tighter acquisition window before returning to `SHOOT`.
- A Shooter that was already in `TRANSITION` cannot be promoted by a setpoint setter; only the
  normal Shooter state transition can enter `SHOOT`.

This makes `currentState` describe readiness for the current target at the moment Superstructure
checks it. A meaningful target change still stops Hopper feeding until the mechanism reaches the
new target. A small target refresh that is already satisfied no longer creates a false transition.

Superstructure applies each distance-based target atomically. In neutral shoot and purge zones it
passes the interpolated flywheel RPM and the 1.25-rotation hood override in one Shooter setpoint
update, instead of briefly applying the interpolated hood position first. This prevents an
intermediate, non-commanded target from affecting readiness.

No motor configuration, current limit, PID value, control mode, alignment rule, or target value
changes.

## Alternatives Rejected

1. Gate Hopper directly on raw flywheel and hood readiness. This bypasses the existing Shooter
   state-machine contract and could feed before the Shooter has completed its `SHOOT` transition.
2. Never leave `SHOOT` when a setpoint changes. This could continue feeding after a large target
   change while the flywheel or hood is not ready.
3. Ignore target changes below an arbitrary epsilon. This can leave the commanded target stale and
   introduces another tuning threshold instead of using the existing physical readiness windows.

## Verification

Follow test-driven development with two regression levels:

- Shooter unit tests prove the inclusive +/-60 RPM acquisition bounds, inclusive -120/+60 RPM
  maintenance bounds, and the 0.125-rotation hood bound. They also prove that losing maintenance
  readiness immediately returns to `TRANSITION` with the 6 V kicker preparation output.
- A changed setpoint already inside the maintenance window remains in `SHOOT`. A large change
  enters `TRANSITION` and cannot return until the tighter acquisition window is met.
- A Superstructure scheduler test reproduces the neutral-zone target refresh and proves that a
  ready Shooter allows Hopper to request `INDEX_TO_SHOOTER` across a complete scheduler loop.

Run both focused test classes, then the full project verification task, formatting checks, and
`git diff --check`.

## Acceptance Criteria

1. Shooter enters `SHOOT` only when flywheel speed is within +/-60 RPM and hood position is within
   +/-0.125 rotations of their targets.
2. Shooter remains in `SHOOT` while flywheel speed is between target -120 RPM and target +60 RPM
   and hood position remains within +/-0.125 rotations.
3. Leaving the maintenance window immediately returns Shooter to `TRANSITION`, commands the 6 V
   kicker preparation output, and blocks Hopper feeding.
4. A same-loop target refresh that remains inside the maintenance window does not create a false
   transition, and neutral-zone targets are applied atomically.
5. Hopper enters `INDEX_TO_SHOOTER` when Shooter is ready and any required alignment condition is
   satisfied.
6. Existing mechanism configuration and target values remain unchanged.
