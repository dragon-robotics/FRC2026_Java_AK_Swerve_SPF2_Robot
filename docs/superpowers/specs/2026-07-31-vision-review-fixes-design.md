# Vision PR Review Fixes Design

## Context

Copilot left four unresolved review threads on PR #6. Two threads describe the same
`GyroIOPigeon2` refresh-array allocation, leaving three distinct issues. None changes the
approved vision behavior, solver order, camera geometry, estimator fusion, or CAN status rates.

## Approved changes

### Package-aligned simulation harness

Move `VisionSimulationHarness.java` from the `vision/sim` source directory to the `vision`
source directory while retaining `package frc.robot.subsystems.vision`. The harness needs
package-private access to vision simulation internals. The three executable simulation tests stay
in `frc.robot.subsystems.vision.sim` and remain isolated by their `@Tag("vision-sim")` annotations,
so the move does not change Gradle test selection.

### Coherent public drive-binding API

Move `VisionMeasurementConsumer` into its own public source file. Keep the existing type name and
method signature so `VisionDriveBindings` and all callers retain the same data flow. This makes the
public record constructor and `measurementConsumer()` accessor usable outside the vision package
without widening any other vision implementation details.

### Cached Pigeon refresh group

Construct the four-element Pigeon refresh-signal array once in `GyroIOPigeon2` and retain it in a
final field. `updateInputs()` passes that cached array to `BaseStatusSignal.refreshAll(...)` on every
loop. Signal membership, refresh-before-read ordering, update frequencies, and bus optimization
remain unchanged.

## Verification

- Prove the binding API problem with an outside-package compile test, then make it pass.
- Preserve the existing gyro telemetry contract test and inspect compiled bytecode to confirm
  `updateInputs()` reads the cached field instead of calling the allocating factory.
- Run the normal tests, isolated Photon simulation tests, Spotless checks, and the complete Gradle
  `check` lifecycle.
- Keep GitHub replies and thread resolution out of scope unless separately requested.
