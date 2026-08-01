# Vision PR Review Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct all three distinct Copilot review findings on PR #6 without changing vision or gyro behavior.

**Architecture:** Preserve the existing package boundaries and runtime contracts. Expose only the callback type already present in the public record ABI, relocate the test harness without changing its Java package, and reuse one immutable-reference Pigeon signal array in the periodic loop.

**Tech Stack:** Java 17, WPILib 2026, CTRE Phoenix 6, JUnit 5, GradleRIO, AdvantageKit, PhotonVision.

## Global Constraints

- Keep the four-camera geometry, solver precedence, filtering, consensus, initialization, and reseed behavior unchanged.
- Keep Pigeon yaw, yaw-rate, pitch, and roll refresh membership and update frequencies unchanged.
- Keep simulation tests isolated with the existing `vision-sim` JUnit tag.
- Do not reply to or resolve GitHub review threads without explicit authorization.

---

### Task 1: Make the measurement callback publicly usable

**Files:**
- Create: `src/main/java/frc/robot/subsystems/vision/VisionMeasurementConsumer.java`
- Modify: `src/main/java/frc/robot/subsystems/vision/VisionDriveBindings.java:3-18`
- Create: `src/test/java/frc/robot/VisionDriveBindingsPublicApiTest.java`

**Interfaces:**
- Consumes: `VisionDriveBindings(VisionMeasurementConsumer, Consumer<Pose2d>, Supplier<Pose2d>, DoubleFunction<Optional<Pose2d>>, Supplier<ChassisSpeeds>, DoubleSupplier, DoubleSupplier)`.
- Produces: public `VisionMeasurementConsumer.accept(Pose2d, double, Matrix<N3, N1>)` with the existing signature.

- [ ] **Step 1: Write the outside-package compile test**

```java
package frc.robot;

import static org.junit.jupiter.api.Assertions.assertSame;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.vision.VisionDriveBindings;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class VisionDriveBindingsPublicApiTest {
  @Test
  void publicConstructorAndMeasurementAccessorAreUsableOutsideVisionPackage() {
    AtomicReference<Pose2d> acceptedPose = new AtomicReference<>();
    VisionDriveBindings bindings =
        new VisionDriveBindings(
            (pose, timestampSeconds, standardDeviations) -> acceptedPose.set(pose),
            pose -> {},
            Pose2d::new,
            timestampSeconds -> Optional.of(new Pose2d()),
            ChassisSpeeds::new,
            () -> 0.0,
            () -> 0.0);
    Pose2d expected = new Pose2d(1.0, 2.0, new Rotation2d(0.3));

    bindings.measurementConsumer().accept(expected, 1.25, VecBuilder.fill(1.0, 1.0, 1.0));

    assertSame(expected, acceptedPose.get());
  }
}
```

- [ ] **Step 2: Run the test and verify the accessibility failure**

Run: `.\gradlew.bat test --tests "frc.robot.VisionDriveBindingsPublicApiTest" --console=plain`

Expected: `compileTestJava` fails because `VisionMeasurementConsumer` is not public.

- [ ] **Step 3: Move the callback into a public source file**

```java
package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

@FunctionalInterface
public interface VisionMeasurementConsumer {
  void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations);
}
```

Remove the package-private declaration and now-unused matrix imports from `VisionDriveBindings.java`.

- [ ] **Step 4: Rerun the focused test**

Run: `.\gradlew.bat test --tests "frc.robot.VisionDriveBindingsPublicApiTest" --console=plain`

Expected: `BUILD SUCCESSFUL` and the test passes.

---

### Task 2: Align the simulation harness source path

**Files:**
- Move: `src/test/java/frc/robot/subsystems/vision/sim/VisionSimulationHarness.java`
- To: `src/test/java/frc/robot/subsystems/vision/VisionSimulationHarness.java`
- Modify: `docs/superpowers/plans/2026-07-31-vision-subsystem-port.md`

**Interfaces:**
- Consumes: package-private vision simulation hooks from `frc.robot.subsystems.vision`.
- Produces: the unchanged `frc.robot.subsystems.vision.VisionSimulationHarness` class used by the three tagged simulation tests.

- [ ] **Step 1: Record the current layout mismatch**

Run: `Test-Path src/test/java/frc/robot/subsystems/vision/VisionSimulationHarness.java; Test-Path src/test/java/frc/robot/subsystems/vision/sim/VisionSimulationHarness.java`

Expected: `False`, then `True`.

- [ ] **Step 2: Move the file without changing its package or contents**

Move the source file to the package-aligned path and update its historical path references in the existing implementation plan.

- [ ] **Step 3: Verify layout and simulation compilation**

Run: `Test-Path src/test/java/frc/robot/subsystems/vision/VisionSimulationHarness.java; Test-Path src/test/java/frc/robot/subsystems/vision/sim/VisionSimulationHarness.java`

Expected: `True`, then `False`.

Run: `.\gradlew.bat visionSimulationTest --console=plain`

Expected: all isolated simulation tests pass.

---

### Task 3: Reuse the Pigeon refresh array

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/drive/GyroIOPigeon2.java:25-57`
- Test: `src/test/java/frc/robot/subsystems/drive/GyroVisionTelemetryTest.java`

**Interfaces:**
- Consumes: the existing `createRefreshSignals(yaw, yawVelocity, pitch, roll)` helper.
- Produces: one final `BaseStatusSignal[] refreshSignals` reused by `updateInputs()`.

- [ ] **Step 1: Capture the allocating bytecode before the refactor**

Run: `javap -classpath build/classes/java/main -c -p frc.robot.subsystems.drive.GyroIOPigeon2`

Expected: `updateInputs()` invokes `createRefreshSignals(...)` before `refreshAll(...)`.

- [ ] **Step 2: Cache and reuse the refresh group**

```java
private final BaseStatusSignal[] refreshSignals =
    createRefreshSignals(yaw, yawVelocity, pitch, roll);
```

Replace the periodic factory call with `BaseStatusSignal.refreshAll(refreshSignals)`.

- [ ] **Step 3: Verify the telemetry contract and compiled call path**

Run: `.\gradlew.bat test --tests "frc.robot.subsystems.drive.GyroVisionTelemetryTest" --console=plain`

Expected: `BUILD SUCCESSFUL` and all gyro telemetry tests pass.

Run: `javap -classpath build/classes/java/main -c -p frc.robot.subsystems.drive.GyroIOPigeon2`

Expected: `updateInputs()` loads the `refreshSignals` field and does not invoke `createRefreshSignals(...)`.

---

### Task 4: Verify and publish the review fixes

**Files:**
- Verify all files changed by Tasks 1-3.

**Interfaces:**
- Consumes: the complete `feature/vision` review-fix diff.
- Produces: one scoped review-fix commit pushed to PR #6.

- [ ] **Step 1: Run focused and full verification**

Run: `.\gradlew.bat check --console=plain --rerun-tasks`

Expected: normal and isolated vision tests pass and `BUILD SUCCESSFUL` is printed.

- [ ] **Step 2: Verify diff hygiene and scope**

Run: `git diff --check`

Run: `git status --short`

Expected: only the approved source, test, relocation, and documentation files are changed.

- [ ] **Step 3: Commit and push**

Stage only the approved files, commit as `fix(vision): address PR review feedback`, and push `feature/vision` to its configured upstream. Do not reply to or resolve review threads.
