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

## Design

Keep the existing Superstructure contract: Hopper feeding still requires
`shooter.getCurrentState() == ShooterState.SHOOT`. Do not bypass the Shooter state machine, change
the readiness tolerances, or reorder subsystem periodic execution.

When `Shooter.setSetpoint()` changes a target while the desired state is `SHOOT`, evaluate the most
recent measured flywheel speed and hood position against the new target:

- If both measurements satisfy the existing readiness tolerances, retain or restore `SHOOT`.
- If either measurement is outside its readiness tolerance, enter `TRANSITION` as today.

This makes `currentState` describe readiness for the current target at the moment Superstructure
checks it. A meaningful target change still stops Hopper feeding until the mechanism reaches the
new target. A small target refresh that is already satisfied no longer creates a false transition.

No motor requests, current limits, PID values, control modes, alignment rules, or readiness
tolerances change.

## Alternatives Rejected

1. Gate Hopper directly on raw flywheel and hood readiness. This bypasses the existing Shooter
   state-machine contract and could feed before the Shooter has completed its `SHOOT` transition.
2. Never leave `SHOOT` when a setpoint changes. This could continue feeding after a large target
   change while the flywheel or hood is not ready.
3. Ignore target changes below an arbitrary epsilon. This can leave the commanded target stale and
   introduces another tuning threshold instead of using the existing physical readiness windows.

## Verification

Follow test-driven development with two regression levels:

- A Shooter unit test proves that a changed setpoint already satisfied by the measured mechanism
  remains in `SHOOT`. The existing large-change regression continues to prove that an unsatisfied
  target enters `TRANSITION`.
- A Superstructure scheduler test reproduces the neutral-zone target refresh and proves that a
  ready Shooter allows Hopper to request `INDEX_TO_SHOOTER` across a complete scheduler loop.

Run both focused test classes, then the full project verification task, formatting checks, and
`git diff --check`.

## Acceptance Criteria

1. Shooter state and its logged value no longer disagree with the same-loop Hopper readiness check
   solely because a refreshed target remains within the approved readiness tolerances.
2. Hopper enters `INDEX_TO_SHOOTER` when the ready Shooter and any required alignment condition are
   satisfied.
3. A target change outside either readiness tolerance still puts Shooter in `TRANSITION` and blocks
   Hopper feeding.
4. Existing mechanism configuration and tolerances remain unchanged.
