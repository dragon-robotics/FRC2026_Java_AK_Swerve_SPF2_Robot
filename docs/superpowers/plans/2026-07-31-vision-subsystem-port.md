# Vision Subsystem Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the complete four-camera `spitfire-v2` AprilTag vision stack into this repository's AdvantageKit architecture, including the approved timestamp-coherent consensus and positive-X tag-normal fixes.

**Architecture:** Four REAL, SIM, or REPLAY camera IO implementations publish replayable observations to a target-native `Vision` subsystem. `Vision` validates timestamps before pose-history access, filters observations, selects one timestamp-coherent spatial-consensus winner, and sends that real camera measurement to `Drive`. A shared Photon simulation reads independent drivetrain truth, while vision reseeds reset only the estimator.

**Tech Stack:** Java 17, WPILib/GradleRIO 2026.2.1, AdvantageKit, PhotonLib v2026.3.4, Phoenix 6, JUnit 5, PhotonVision simulation.

## Global Constraints

- Read reference behavior from immutable commit `70cce7cc0ee2c3b53644a582acf5b6bb9be35dd8` in `C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_Swerve_Robot`, not from the moving `spitfire-v2` branch name.
- Preserve the four camera names, ordering, transforms, and standard-deviation factors exactly. Camera order is the final deterministic consensus tie breaker.
- Use `FieldConstants.APTAG_FIELD_LAYOUT`; do not change `FieldConstants.java`, the deployed AprilTag JSON, `Constants.java`, or `Robot.java`.
- Do not add DogLog, Lombok, MegaTag producers, object detection, named test autos, runtime CSV/report generation, the count-only bakeoff, or the reference consensus microbenchmark.
- Capture JVM properties once in `VisionRuntimeConfig`; tests must not mutate process-global system properties.
- Keep FPGA capture timestamps unchanged through pose-history sampling and `addVisionMeasurement`. Do not call CTRE `Utils.fpgaToCurrentTime()`.
- Validate timestamp finiteness, future skew, and age before any pose-history lookup, initialization update, or accepted-snapshot update.
- Normal enabled fusion never resets the estimator. Automatic reseeding runs only while disabled. Manual reseeding retains the reference enabled-only command behavior.
- `Drive.setPose()` resets only the estimator. `Drive.setPoseAndSimulationTruth()` resets estimator and SIM truth. Vision and driver heading recovery continue to call `setPose()`.
- One shared, instance-owned `VisionSystemSim` updates exactly once before all four simulated cameras are read. No static Photon simulation state is permitted.
- Every mutable tag-ID array is defensively copied on input and output, including nested arrays inside observations and snapshots.
- Every transient AdvantageKit output is overwritten with an empty array, empty string, zero, or false when absent; no prior-frame output may remain visible.
- Use `apply_patch` for source edits. Preserve unrelated worktree changes. Run `git status --short` before every commit and stage only the task's files.
- For every task, write the named test first, run the exact focused command, observe the stated failure, implement only that task, rerun the same command, then commit.
- Before claiming completion, invoke `superpowers:verification-before-completion` and use the fresh outputs from Task 11.

---

## File Map

### New production files

- `vendordeps/photonlib.json` — exact PhotonLib v2026.3.4 vendordep from the reference commit.
- `src/main/java/frc/robot/subsystems/vision/VisionConstants.java` — ordered camera configuration plus every approved threshold and simulation property.
- `src/main/java/frc/robot/subsystems/vision/VisionRuntimeConfig.java` — immutable once-captured property parsing and startup-order selection.
- `src/main/java/frc/robot/subsystems/vision/VisionIO.java` — `@AutoLog` camera inputs, immutable observations, target-owned enums, and named replay no-op.
- `src/main/java/frc/robot/subsystems/vision/VisionGeometry.java` — corrected local-positive-X tag-face coplanarity calculation.
- `src/main/java/frc/robot/subsystems/vision/VisionFilter.java` — pure hard-gate and covariance policy.
- `src/main/java/frc/robot/subsystems/vision/VisionConsensus.java` — pure timestamp validation, newest-per-camera reduction, temporal clustering, and spatial selection.
- `src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java` — Photon result draining, distance aggregation, solver ordering, and pose-attempt execution.
- `src/main/java/frc/robot/subsystems/vision/VisionDriveBindings.java` — callback/provider record isolating `Vision` from the concrete drivetrain.
- `src/main/java/frc/robot/subsystems/vision/Vision.java` — AdvantageKit orchestration, initialization, snapshots, reseeding, alerts, and field overlays.
- `src/main/java/frc/robot/subsystems/vision/VisionSimulation.java` — sole owner of one shared `VisionSystemSim` and its independent truth supplier.
- `src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVisionSim.java` — one simulated camera registered with the shared simulation owner.
- `src/main/java/frc/robot/subsystems/drive/DriveSimulationPoseTracker.java` — small `SwerveDriveOdometry` wrapper used by `Drive` for independent SIM truth.
- `src/main/java/frc/robot/VisionConstructionPlan.java` — pure mode-to-IO-kind plan with the immutable ordered camera list.

### Modified production files

- `build.gradle` — isolate JNI/global-state Photon scenarios in `visionSimulationTest` and wire that task into `check`.
- `src/main/java/frc/robot/subsystems/drive/GyroIO.java` — add replayable pitch and roll degrees.
- `src/main/java/frc/robot/subsystems/drive/GyroIOPigeon2.java` — refresh yaw, yaw rate, pitch, and roll; configure pitch/roll at 50 Hz before bus optimization.
- `src/main/java/frc/robot/subsystems/drive/GyroIONavX.java` — populate pitch and roll parity fields.
- `src/main/java/frc/robot/subsystems/drive/Drive.java` — timestamped sampling, tilt accessors, independent SIM truth, and split reset semantics.
- `src/main/java/frc/robot/RobotContainer.java` — construct and retain `Vision`, materialize the mode plan, share SIM ownership, and bind operator Start+Back.

### New normal-test files

- `src/test/java/frc/robot/subsystems/vision/VisionConstantsTest.java`
- `src/test/java/frc/robot/subsystems/vision/VisionRuntimeConfigTest.java`
- `src/test/java/frc/robot/subsystems/vision/VisionIOTest.java`
- `src/test/java/frc/robot/subsystems/vision/VisionGeometryTest.java`
- `src/test/java/frc/robot/subsystems/vision/VisionFilterTest.java`
- `src/test/java/frc/robot/subsystems/vision/VisionConsensusTest.java`
- `src/test/java/frc/robot/subsystems/vision/VisionStrategyOrderTest.java`
- `src/test/java/frc/robot/subsystems/vision/VisionIOPhotonVisionTest.java`
- `src/test/java/frc/robot/subsystems/vision/VisionTest.java`
- `src/test/java/frc/robot/subsystems/vision/VisionLoggingTest.java`
- `src/test/java/frc/robot/subsystems/vision/VisionReplayTest.java`
- `src/test/java/frc/robot/subsystems/vision/VisionInitializationReseedTest.java`
- `src/test/java/frc/robot/subsystems/vision/VisionStartupStrategyComparisonSupport.java`
- `src/test/java/frc/robot/subsystems/vision/VisionStartupStrategyComparatorTest.java`
- `src/test/java/frc/robot/subsystems/drive/DriveSimulationPoseTrackerTest.java`
- `src/test/java/frc/robot/subsystems/drive/DriveVisionSupportTest.java`
- `src/test/java/frc/robot/subsystems/drive/GyroVisionTelemetryTest.java`
- `src/test/java/frc/robot/VisionConstructionPlanTest.java`
- `src/test/java/frc/robot/RobotContainerVisionTest.java`

### New isolated Photon-simulation test files

- `src/test/java/frc/robot/subsystems/vision/VisionSimulationHarness.java`
- `src/test/java/frc/robot/subsystems/vision/sim/VisionSimulationLifecycleTest.java`
- `src/test/java/frc/robot/subsystems/vision/sim/VisionPhotonScenariosTest.java`
- `src/test/java/frc/robot/subsystems/vision/sim/VisionStartupStrategyComparisonTest.java`
- `src/test/java/frc/robot/RobotContainerVisionBindingTest.java`

---

### Task 1: Add the Photon dependency and lock the data/configuration contracts

**Files:**

- Create: `vendordeps/photonlib.json`
- Create: `src/main/java/frc/robot/subsystems/vision/VisionConstants.java`
- Create: `src/main/java/frc/robot/subsystems/vision/VisionRuntimeConfig.java`
- Create: `src/main/java/frc/robot/subsystems/vision/VisionIO.java`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionConstantsTest.java`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionRuntimeConfigTest.java`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionIOTest.java`

**Interfaces:**

- Consumes: existing AdvantageKit `@AutoLog`/`LogTable`, WPILib geometry, and the immutable reference vendordep.
- Produces: `VisionConstants.CameraConfig`, `VisionRuntimeConfig`, `StartupStrategyOrder`, `TagDistanceConfidenceMode`, `VisionIO`, `VisionIOInputs`, defensive camera-level tag-ID accessors, `TargetObservation`, `PoseObservation`, `PoseObservationType`, and `PoseSolveStrategy` used by every later task.

- [ ] Read `superpowers:test-driven-development/SKILL.md` and its `writing-good-tests.md` completely before adding the first test.

- [ ] Add `VisionConstantsTest` first. Assert the ordered names, all four inch-to-meter translations, all four rotations, four `1.0` camera factors, and the locked values below.

```java
assertEquals(
    List.of(
        "AprilTagPoseEstCameraF",
        "AprilTagPoseEstCameraR",
        "AprilTagPoseEstCameraB",
        "AprilTagPoseEstCameraL"),
    VisionConstants.CAMERAS.stream().map(CameraConfig::name).toList());
assertAll(
    () -> assertEquals(0.020, VisionConstants.MAX_FUTURE_TIMESTAMP_SECONDS, 1e-12),
    () -> assertEquals(0.500, VisionConstants.MAX_OBSERVATION_AGE_SECONDS, 1e-12),
    () -> assertEquals(0.040, VisionConstants.MAX_CONSENSUS_TIMESTAMP_SPAN_SECONDS, 1e-12),
    () -> assertEquals(0.45, VisionConstants.CONSENSUS_RADIUS_METERS, 1e-12),
    () -> assertEquals(1e9, VisionConstants.HEADING_STD_DEV, 1.0));
```

`CameraConfig` is:

```java
public record CameraConfig(String name, Transform3d robotToCamera, double stdDevFactor) {}
```

Use these exact transforms:

```java
new CameraConfig("AprilTagPoseEstCameraF", transform(-11.152, -7.579, 20.930, 0.0, -15.0, 0.0), 1.0)
new CameraConfig("AprilTagPoseEstCameraR", transform(-8.387, -13.355, 15.931, 0.0, -12.0, -90.0), 1.0)
new CameraConfig("AprilTagPoseEstCameraB", transform(-9.164, 12.500, 20.839, 0.0, -15.0, 180.0), 1.0)
new CameraConfig("AprilTagPoseEstCameraL", transform(-8.387, 13.355, 15.931, 0.0, -12.0, 90.0), 1.0)
```

Define and test the complete constant set so later tasks do not invent values:

| Constant | Value |
|---|---:|
| `MAX_AMBIGUITY` | `0.2` |
| `MAX_Z_ERROR_METERS` | `0.5` |
| `MAX_TARGET_PREFILTER_DISTANCE_METERS` | `8.0` |
| `MAX_CONFIDENCE_DISTANCE_METERS` | `5.5` |
| `MAX_POSE_INNOVATION_METERS` | `2.5` |
| `CONSENSUS_RADIUS_METERS` | `0.45` |
| `MAX_FUTURE_TIMESTAMP_SECONDS` | `0.020` |
| `MAX_OBSERVATION_AGE_SECONDS` | `0.500` |
| `MAX_CONSENSUS_TIMESTAMP_SPAN_SECONDS` | `0.040` |
| `SNAPSHOT_MAX_AGE_SECONDS` | `0.5` |
| `DISABLED_AUTO_RESEED_MIN_INTERVAL_SECONDS` | `0.5` |
| `DISABLED_AUTO_RESEED_DELTA_METERS` | `0.25` |
| `DISABLED_AUTO_RESEED_MIN_TAG_COUNT` | `2` |
| `MULTITAG_INIT_STABLE_POSES_REQUIRED` | `5` |
| `MULTITAG_INIT_MAX_TRANSLATION_DELTA_METERS` | `0.20` |
| `MULTITAG_INIT_MAX_HEADING_DELTA_DEGREES` | `10.0` |
| `LINEAR_STD_DEV_BASELINE` | `0.02` |
| `HEADING_STD_DEV` | `1e9` |
| `AIMING_STD_DEV_FACTOR` | `0.6` |
| `VULNERABLE_GEOMETRY_STD_DEV_FACTOR` | `5.0` |
| `CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC` | `0.5` |
| `TRIG_MAX_ANGULAR_RATE_RAD_PER_SEC` | `1.0` |
| `HYBRID_TRANSLATION_SPEED_THRESHOLD_MPS` | `0.5` |
| `CONSTRAINED_HEADING_SCALE_FACTOR` | `0.2` |
| `COPLANAR_ANGLE_THRESHOLD_DEGREES` | `15.0` |
| `SIM_CAMERA_WIDTH_PIXELS` / `SIM_CAMERA_HEIGHT_PIXELS` | `800` / `600` |
| `SIM_CAMERA_DIAGONAL_FOV_DEGREES` | `72.0` |
| `SIM_CAMERA_CALIBRATION_ERROR_MEAN` / `STD_DEV` | `0.38` / `0.1` |
| `SIM_CAMERA_FPS` | `60.0` |
| `SIM_CAMERA_AVERAGE_LATENCY_MS` / `STD_DEV_MS` | `10.0` / `5.0` |

Initialize `DEFAULT_STARTUP_STRATEGY_ORDER` to `StartupStrategyOrder.CONSTRAINED_SECOND`, the approved deterministic tie preference. Task 9 changes it only if the passing comparison selects `REFERENCE`.

- [ ] Add `VisionRuntimeConfigTest`. Use `Properties` or a `Function<String, String>` lookup; never call `System.setProperty`. Cover defaults, valid values, case/whitespace normalization, a partially valid strategy list, an all-invalid list, blank strategy order, invalid numeric/boolean/distance values, and direct startup-order injection.

Lock these target-owned types as public enums nested in `VisionRuntimeConfig` so `VisionConstants` and `RobotContainer` can use them without adding files:

```java
record VisionRuntimeConfig(
    double maxAbsTiltDegrees,
    boolean applyCoplanarPenalty,
    String strategyMode,
    List<VisionIO.PoseSolveStrategy> configuredStrategyOrder,
    boolean explicitStrategyOrder,
    TagDistanceConfidenceMode tagDistanceConfidenceMode,
    StartupStrategyOrder startupStrategyOrder) {
  public enum StartupStrategyOrder { REFERENCE, CONSTRAINED_SECOND }
  public enum TagDistanceConfidenceMode { ALL_TAG_AVERAGE, MAX_TAG_DISTANCE }
  static VisionRuntimeConfig fromSystemProperties();
  static VisionRuntimeConfig parse(
      Function<String, String> propertyLookup, StartupStrategyOrder startupOrder);
}
```

Defaults are `8.0`, `true`, `HYBRID`, no explicit order, `ALL_TAG_AVERAGE`, and `VisionConstants.DEFAULT_STARTUP_STRATEGY_ORDER`. An explicit list ignores unsupported tokens while preserving supported-token order. An explicit all-invalid list falls back to `MULTI_TAG_PNP_ON_COPROCESSOR`, `CONSTRAINED_SOLVEPNP`, `PNP_DISTANCE_TRIG_SOLVE`, `LOWEST_AMBIGUITY`.

- [ ] Add `VisionIOTest` with these independent contracts:

  1. `PoseObservation` copies tag IDs in its constructor and accessor.
  2. `Accepted` input arrays can be changed without mutating a previously constructed observation.
  3. Camera-level tag IDs default to empty, treat null as empty, and are copied on setter input and getter output.
  4. `VisionIO.NoOp("camera")` always overwrites camera name, disconnected state, zero target angles, and empty observation/tag arrays.
  5. `VisionIOInputsAutoLogged` losslessly round-trips a `PoseObservation[]` containing `Pose3d`, both enums, and nested `int[]` through a real `LogTable` using supported `PoseObservationLog[]` fields plus an `int[][]` sidecar. Mismatched or missing sidecar rows reconstruct as empty arrays without corrupting other observations.
  6. `VisionIOInputsAutoLogged` directly round-trips the separate aggregate camera-level `int[]` through a real `LogTable` without unsupported-field diagnostics.

Use this round-trip shape so AdvantageKit serialization is proven before the rest of the port depends on it:

```java
VisionIOInputsAutoLogged source = new VisionIOInputsAutoLogged();
source.cameraName = "AprilTagPoseEstCameraF";
source.setPoseObservations(new PoseObservation[] {observation});
source.setTagIds(new int[] {3, 7});
LogTable table = new LogTable(0);
source.toLog(table);

VisionIOInputsAutoLogged replayed = new VisionIOInputsAutoLogged();
replayed.fromLog(table);
assertEquals(observation.strategy(), replayed.getPoseObservations()[0].strategy());
assertArrayEquals(new int[] {3, 7}, replayed.getPoseObservations()[0].tagIds());
assertArrayEquals(new int[] {3, 7}, replayed.getTagIds());
```

The IO contract is:

```java
record TargetObservation(Rotation2d tx, Rotation2d ty) {
  static final TargetObservation NONE =
      new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
}

enum PoseObservationType {
  MEGATAG_1,
  MEGATAG_2,
  PHOTONVISION,
  PHOTONVISION_MULTITAG_COPROCESSOR
}

enum PoseSolveStrategy {
  MULTI_TAG_PNP_ON_COPROCESSOR,
  CONSTRAINED_SOLVEPNP,
  PNP_DISTANCE_TRIG_SOLVE,
  LOWEST_AMBIGUITY,
  UNKNOWN
}

record PoseObservation(
    double timestampSeconds,
    Pose3d pose,
    double ambiguity,
    int tagCount,
    double confidenceDistanceMeters,
    PoseObservationType type,
    PoseSolveStrategy strategy,
    int[] tagIds) {}

record PoseObservationLog(
    double timestampSeconds,
    Pose3d pose,
    double ambiguity,
    int tagCount,
    double confidenceDistanceMeters,
    PoseObservationType type,
    PoseSolveStrategy strategy) {}

@AutoLog
class VisionIOInputs {
  public String cameraName = "";
  public boolean connected = false;
  public TargetObservation latestTargetObservation = TargetObservation.NONE;
  public PoseObservationLog[] poseObservationLogs = new PoseObservationLog[0];
  public int[][] poseObservationTagIds = new int[0][];
  public int[] tagIds = new int[0];
  public void setPoseObservations(PoseObservation[] poseObservations) {}
  public PoseObservation[] getPoseObservations() { return new PoseObservation[0]; }
  public void setTagIds(int[] tagIds) {}
  public int[] getTagIds() { return new int[0]; }
}

String getCameraName();
default void updateInputs(VisionIOInputs inputs) {}
default void markVisionInitializationComplete() {}
```

`PoseObservation` fields, in order, are capture timestamp, field-relative robot `Pose3d`, nonnegative mean ambiguity, positive tag count, confidence distance, `PoseObservationType`, successful `PoseSolveStrategy`, and contributing tag IDs. `PoseSolveStrategy` contains the four supported Photon strategies and `UNKNOWN`. The separate camera-level IDs are the deduplicated union from successfully emitted observations in the current update; accessors defensively copy them and null input clears them.

- [ ] Run the red gate:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.vision.VisionConstantsTest" --tests "frc.robot.subsystems.vision.VisionRuntimeConfigTest" --tests "frc.robot.subsystems.vision.VisionIOTest"
```

Expected: `compileTestJava` fails because the vision contract classes and generated `VisionIOInputsAutoLogged` do not exist.

- [ ] Add `vendordeps/photonlib.json` byte-for-byte from reference commit `70cce7c` and implement only the constants, runtime parser, and IO types. Use `List.copyOf`, canonical record constructors, and copied array accessors to make each returned value immutable.

- [ ] Rerun the focused gate. Expected: `BUILD SUCCESSFUL`, including the real lossless AdvantageKit record-array-and-sidecar replay round trip and direct aggregate `int[]` replay.

- [ ] Inspect generated `build/generated/sources/annotationProcessor/java/main/frc/robot/subsystems/vision/VisionIOInputsAutoLogged.java` and confirm it uses `LogTable.put/get` for the supported record array, `int[][]` sidecar, and aggregate `int[]` rather than passing `PoseObservation[]` to `RecordStruct`.

- [ ] Commit only Task 1:

```powershell
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' add vendordeps/photonlib.json src/main/java/frc/robot/subsystems/vision/VisionConstants.java src/main/java/frc/robot/subsystems/vision/VisionRuntimeConfig.java src/main/java/frc/robot/subsystems/vision/VisionIO.java src/test/java/frc/robot/subsystems/vision/VisionConstantsTest.java src/test/java/frc/robot/subsystems/vision/VisionRuntimeConfigTest.java src/test/java/frc/robot/subsystems/vision/VisionIOTest.java
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' commit -m "feat(vision): define runtime configuration and IO contract"
```

---

### Task 2: Implement hard filtering, covariance, and corrected tag geometry

**Files:**

- Create: `src/main/java/frc/robot/subsystems/vision/VisionGeometry.java`
- Create: `src/main/java/frc/robot/subsystems/vision/VisionFilter.java`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionGeometryTest.java`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionFilterTest.java`

**Interfaces:**

- Consumes: `VisionConstants.CameraConfig`, `VisionRuntimeConfig`, `VisionIO.PoseObservation`, and an injected `AprilTagFieldLayout` from Task 1.
- Produces: `VisionGeometry.areTagsCoplanar`, `VisionGeometry.angleBetweenTagNormalsRadians`, `VisionFilter.rejectionReason`, and `VisionFilter.standardDeviations` for Photon strategy selection and `Vision` orchestration.

- [ ] Add `VisionGeometryTest` with an injected `AprilTagFieldLayout`, not the production field. Build tag rotations whose local positive-X normals are equal, exactly 15 degrees apart, just beyond 15 degrees, opposite, and independent of local positive Z. Cover one tag, unknown first ID, and later unknown IDs. Add two production-layout regressions using `FieldConstants.APTAG_FIELD_LAYOUT`: two known tags on one Hub face must be coplanar, while known tags on different Hub faces must not be coplanar.

Lock the pure API:

```java
static boolean areTagsCoplanar(AprilTagFieldLayout layout, int[] tagIds);
static double angleBetweenTagNormalsRadians(Rotation3d first, Rotation3d second);
```

The implementation rotates `new Translation3d(1.0, 0.0, 0.0)` by each known tag rotation. `<= 15 degrees` is coplanar. One known tag is coplanar. A missing first tag is conservatively coplanar; later missing tags are skipped.

- [ ] Add parameterized `VisionFilterTest` cases for every gate and equality boundary:

| Gate | Reject | Accept boundary |
|---|---:|---:|
| tag count | `0` | `1` |
| absolute Z | `> 0.5 m` | `0.5 m` |
| field X/Y | outside layout | exactly `0`, field length, or field width |
| single-tag ambiguity | `> 0.2` | `0.2` |
| confidence distance | `> 5.5 m` or positive infinity | `5.5 m` |
| absolute pitch/roll | `> configured max` | configured max |
| enabled innovation | `> 2.5 m` | `2.5 m` |
| disabled innovation | never rejected by this gate | any finite distance |

Use stable reason constants: `NO_TAGS`, `Z_OUT_OF_RANGE`, `OUTSIDE_FIELD`, `HIGH_AMBIGUITY`, `TAG_DISTANCE_TOO_LARGE`, `TILT_UNSTABLE`, and `POSE_INNOVATION_TOO_LARGE`.

Lock the filter API:

```java
static Optional<String> rejectionReason(
    PoseObservation observation,
    AprilTagFieldLayout layout,
    double pitchDegrees,
    double rollDegrees,
    boolean disabled,
    Pose2d referencePose,
    VisionRuntimeConfig config);

static Matrix<N3, N1> standardDeviations(
    PoseObservation observation,
    CameraConfig camera,
    boolean aiming,
    AprilTagFieldLayout layout,
    VisionRuntimeConfig config);
```

- [ ] Add covariance assertions for distance `2.0`, two tags, camera factor `1.0`: base translation standard deviation is `0.02 * 2^2 / 2 = 0.04`. Assert aiming produces `0.024`; single-tag or enabled coplanarity penalty multiplies by `5`; noncoplanar multitag does not; disabled penalty does not; heading is `1e9`; and translation floors at `1e-6`.

Assert the reference-compatible distance normalization exactly:

```java
double effectiveDistance = rawDistance > 0.0 ? rawDistance : 5.5;
```

This makes zero, negative, and NaN use `5.5`; positive infinity is rejected before covariance is computed.

- [ ] Run the red gate:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.vision.VisionGeometryTest" --tests "frc.robot.subsystems.vision.VisionFilterTest"
```

Expected: compilation fails because `VisionGeometry` and `VisionFilter` do not exist.

- [ ] Implement the two pure classes. Keep timestamp validity outside `VisionFilter`; Task 3 owns timestamp policy so `Vision` can invoke it before sampling drive history.

- [ ] Rerun the focused gate. Expected: `BUILD SUCCESSFUL` with the positive-X regression proving the old positive-Z calculation would fail.

- [ ] Commit only Task 2:

```powershell
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' add src/main/java/frc/robot/subsystems/vision/VisionGeometry.java src/main/java/frc/robot/subsystems/vision/VisionFilter.java src/test/java/frc/robot/subsystems/vision/VisionGeometryTest.java src/test/java/frc/robot/subsystems/vision/VisionFilterTest.java
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' commit -m "feat(vision): add pose filtering and corrected tag geometry"
```

---

### Task 3: Implement timestamp-coherent temporal and spatial consensus

**Files:**

- Create: `src/main/java/frc/robot/subsystems/vision/VisionConsensus.java`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionConsensusTest.java`

**Interfaces:**

- Consumes: `VisionIO.PoseObservation`, `Pose2d`, and `Matrix<N3, N1>` values produced by Tasks 1–2.
- Produces: `VisionConsensus.Candidate`, `RejectedCandidate`, `TemporalSelection`, `timestampRejectionReason`, `selectTimestampCoherent`, and `selectSpatialConsensus` for Task 6.

- [ ] Add `VisionConsensusTest` with builders that expose camera index, configured name, timestamp, pose, linear standard deviation, and innovation. Lock these records:

```java
record Candidate(
    int cameraIndex,
    String cameraName,
    PoseObservation observation,
    Pose2d visionPose,
    Matrix<N3, N1> standardDeviations,
    double innovationMeters) {
  double linearStdDevMeters();
}

record RejectedCandidate(Candidate candidate, String reason) {}
record TemporalSelection(
    List<Candidate> selectedCandidates,
    List<RejectedCandidate> rejectedCandidates) {}
```

- [ ] Test timestamp validity independently before temporal clustering:

```java
static Optional<String> timestampRejectionReason(double timestampSeconds, double nowSeconds);
```

Assert nonfinite timestamps return `INVALID_TIMESTAMP`; `now + 0.020` and `now - 0.500` pass; values one nanosecond beyond return `FUTURE_TIMESTAMP` and `STALE_TIMESTAMP`.

- [ ] Test newest-per-camera reduction. A newer passing frame supersedes every older frame from that camera with `SUPERSEDED_BY_NEWER_CAMERA_FRAME`. Equal timestamps keep the first decoded candidate and supersede the later candidate. A stale or future frame never suppresses a valid frame because validity runs first.

- [ ] Test inclusive temporal windows through:

```java
static TemporalSelection selectTimestampCoherent(
    List<Candidate> validCandidates, double nowSeconds);
```

Cover no candidates, one candidate, a `0.040` second span, a span just beyond `0.040`, and a stale backlog that would have outvoted current cameras in the reference implementation.

- [ ] Pin every deterministic temporal tie breaker in this order:

  1. More distinct cameras.
  2. Newer maximum capture timestamp.
  3. Lower sum of linear standard deviations.
  4. Lexicographically lower sorted camera-index vector.

Every valid retained candidate outside the winning window is rejected as `TIMESTAMP_CLUSTER_NOT_SELECTED`.

- [ ] Test spatial selection through:

```java
static Optional<Candidate> selectSpatialConsensus(List<Candidate> coherentCandidates);
```

Cover one candidate, largest neighborhood, exactly `0.45 m`, just beyond `0.45 m`, equal neighborhood size, the reference quality score tie breaker, and final camera-order stability. The quality score starts with the candidate's linear standard deviation and, for every neighbor at distance `<= 0.45 m` including itself, adds `distanceMeters + neighbor.linearStdDevMeters()`. Lower quality wins. Assert the winner is the original candidate object and preserves its real pose, timestamp, and covariance; no averaging is allowed.

- [ ] Run the red gate:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.vision.VisionConsensusTest"
```

Expected: compilation fails because `VisionConsensus` does not exist.

- [ ] Implement timestamp validation, stable newest-per-camera reduction, inclusive-window enumeration, deterministic temporal comparison, and the reference spatial neighborhood/quality score. Never use unordered-map iteration as a tie breaker.

- [ ] Rerun the focused gate. Expected: `BUILD SUCCESSFUL` and the backlog regression selects the current multi-camera cluster.

- [ ] Commit only Task 3:

```powershell
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' add src/main/java/frc/robot/subsystems/vision/VisionConsensus.java src/test/java/frc/robot/subsystems/vision/VisionConsensusTest.java
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' commit -m "feat(vision): add timestamp-coherent camera consensus"
```

---

### Task 4: Port Photon result decoding and the complete strategy chain

**Files:**

- Create: `src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionStrategyOrderTest.java`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionIOPhotonVisionTest.java`

**Interfaces:**

- Consumes: `CameraConfig`, `VisionRuntimeConfig`, `VisionGeometry.areTagsCoplanar`, every `VisionIO` data type from Tasks 1–2, and `VisionIOInputs.setTagIds()` for the aggregate camera-level IDs.
- Produces: `VisionIOPhotonVision`, its `CameraSource` and `HeadingProvider` seams, deduplicated camera-level tag IDs from successfully emitted observations, deterministic strategy-order helpers, and `markVisionInitializationComplete()` behavior for Tasks 6, 8, and 10.

- [ ] Add `VisionStrategyOrderTest` and pin both startup chains exactly:

```java
REFERENCE = List.of(MULTI_TAG_PNP_ON_COPROCESSOR, PNP_DISTANCE_TRIG_SOLVE,
    CONSTRAINED_SOLVEPNP, LOWEST_AMBIGUITY);
CONSTRAINED_SECOND = List.of(MULTI_TAG_PNP_ON_COPROCESSOR, CONSTRAINED_SOLVEPNP,
    PNP_DISTANCE_TRIG_SOLVE, LOWEST_AMBIGUITY);
```

- [ ] Pin every post-initialization HYBRID branch through a package-private pure method:

```java
static List<PoseSolveStrategy> hybridStrategyOrder(
    int visibleTargetCount,
    boolean coplanarTargetSet,
    double linearSpeedMetersPerSecond,
    double angularRateRadPerSecond);
```

Assert this full table:

| State | Expected order |
|---|---|
| multitag, `|omega| > 0.5` | multi, trig, lowest |
| single-tag, `|omega| > 0.5` | trig, multi, lowest |
| multitag coplanar, `|omega| <= 0.5` | constrained, multi, trig, lowest |
| multitag noncoplanar, `|omega| <= 0.5` | multi, trig, constrained, lowest |
| single-tag, speed `> 0.5`, `|omega| <= 0.5` | trig, multi, constrained, lowest |
| single-tag, speed `<= 0.5`, `|omega| <= 0.5` | trig, constrained, multi, lowest |

At exactly `0.5 rad/s`, constrained remains available. At exactly `1.0 rad/s`, trig remains available; just beyond `1.0`, the trig attempt returns empty. The order may still contain trig because availability is enforced by the attempt method.

- [ ] Test runtime strategy selection after initialization:

  - A nonblank explicit parsed order wins over mode selection.
  - HYBRID uses the table.
  - A non-HYBRID mode without an explicit order uses constrained, multi, trig, lowest.
  - An all-invalid explicit property uses multi, constrained, trig, lowest.
  - `markVisionInitializationComplete()` changes startup selection to configured/HYBRID selection exactly once.

- [ ] Add `VisionIOPhotonVisionTest` around explicit package-private seams rather than network-table timing. Lock these collaborators:

```java
interface CameraSource {
  String name();
  boolean isConnected();
  List<PhotonPipelineResult> getAllUnreadResults();
  Optional<Matrix<N3, N3>> cameraMatrix();
  Optional<Matrix<N8, N1>> distortionCoefficients();
}

interface HeadingProvider {
  Optional<Rotation2d> headingAt(double fpgaTimestampSeconds);
  Optional<Pose3d> seedPoseAt(double fpgaTimestampSeconds);
  double angularRateRadPerSecond();
  double linearSpeedMetersPerSecond();
}
```

The public constructor wraps a real `PhotonCamera`; the package-private constructor accepts `CameraSource`, `PhotonPoseEstimator`, and `HeadingProvider`. Keep `attemptStrategy(PoseSolveStrategy, PhotonPipelineResult)` package-private or protected so a test subclass can return scripted `EstimatedRobotPose` values without Mockito.

- [ ] Cover all-unread behavior with a fake source returning three results in capture order and a scripted strategy attempt. Assert three emitted observations, original timestamps, per-frame successful strategies, latest target angles from the last processed result, and deduplicated camera-level tag IDs from successfully emitted observations only.

- [ ] Cover overwrite behavior in two consecutive calls: first populate targets/observations/tag IDs, then return no unread results and assert zero target angles plus empty arrays. Return `connected=false` with buffered results and assert those results are still drained and emitted.

- [ ] Cover result arithmetic using real `PhotonTrackedTarget` values:

  - Nonpositive fiducial IDs are excluded.
  - Ambiguity is `mean(max(0.0, targetAmbiguity))` across contributing positive IDs.
  - `ALL_TAG_AVERAGE` divides total usable distance by usable distance sample count.
  - `MAX_TAG_DISTANCE` selects the farthest usable distance.
  - A missing transform contributes an ID and ambiguity but no distance sample.
  - No usable distance sample returns positive infinity.
  - Any target with a non-null transform at or inside `8.0 m` defeats the coarse all-far prefilter; all null/far transforms skip solving.

- [ ] Verify each attempt contract:

  - Multi calls `estimateCoprocMultiTagPose`.
  - Lowest ambiguity calls `estimateLowestAmbiguityPose`.
  - Trig requires the heading provider, `|omega| <= 1.0`, and a pose-history heading at the exact capture timestamp, then calls `addHeadingData` before estimating.
  - Constrained requires the provider, `|omega| <= 0.5`, heading, calibration, and distortion. It seeds from lowest-ambiguity pose first, otherwise timestamped drivetrain pose, and uses heading scale `0.2`.
  - The first successful strategy stops the chain and is stored as the target-owned enum on the observation.

- [ ] Run the red gate:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.vision.VisionStrategyOrderTest" --tests "frc.robot.subsystems.vision.VisionIOPhotonVisionTest"
```

Expected: compilation fails because `VisionIOPhotonVision` does not exist.

- [ ] Implement the adapter by semantically porting `VisionIOPhotonVision.java` from reference commit `70cce7c`. Replace static property reads with the captured `VisionRuntimeConfig`, replace Photon enums at the IO boundary with target-owned enums, remove DogLog, and retain all unread results rather than truncating to `MAX_RESULTS_PER_UPDATE`.

- [ ] Rerun the focused gate. Expected: `BUILD SUCCESSFUL`, including disconnected-buffer draining and second-loop stale-field clearing.

- [ ] Commit only Task 4:

```powershell
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' add src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java src/test/java/frc/robot/subsystems/vision/VisionStrategyOrderTest.java src/test/java/frc/robot/subsystems/vision/VisionIOPhotonVisionTest.java
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' commit -m "feat(vision): add Photon solver strategy chain"
```

---

### Task 5: Add gyro telemetry, timestamped drive access, and independent SIM truth

**Files:**

- Create: `src/main/java/frc/robot/subsystems/drive/DriveSimulationPoseTracker.java`
- Modify: `src/main/java/frc/robot/subsystems/drive/GyroIO.java`
- Modify: `src/main/java/frc/robot/subsystems/drive/GyroIOPigeon2.java`
- Modify: `src/main/java/frc/robot/subsystems/drive/GyroIONavX.java`
- Modify: `src/main/java/frc/robot/subsystems/drive/Drive.java`
- Test: `src/test/java/frc/robot/subsystems/drive/DriveSimulationPoseTrackerTest.java`
- Test: `src/test/java/frc/robot/subsystems/drive/DriveVisionSupportTest.java`
- Test: `src/test/java/frc/robot/subsystems/drive/GyroVisionTelemetryTest.java`

**Interfaces:**

- Consumes: existing `Drive`, `GyroIO`, `ModuleIO`, `SwerveDrivePoseEstimator`, and odometry sample arrays.
- Produces: pitch/roll fields, `DriveSimulationPoseTracker`, `Drive.samplePoseAt`, `Drive.getPitchDegrees`, `Drive.getRollDegrees`, `Drive.isPitchRollStableForVision`, `Drive.getSimulationPose`, and `Drive.setPoseAndSimulationTruth` for Tasks 6, 8, and 10.

- [ ] Add `GyroVisionTelemetryTest`. Assert `GyroIOInputs` defaults pitch and roll to `0.0`, and a no-op SIM gyro leaves them explicitly zero. Add package-private Pigeon helpers and assert the refresh group is exactly yaw, yaw velocity, pitch, roll in that order and the pitch/roll configured update rate is `50.0 Hz`.

Use real uninitialized CTRE `StatusSignal` instances, following existing motor-IO config tests:

```java
assertArrayEquals(
    new BaseStatusSignal[] {yaw, yawVelocity, pitch, roll},
    GyroIOPigeon2.createRefreshSignals(yaw, yawVelocity, pitch, roll));
assertEquals(50.0, GyroIOPigeon2.createStatusFrequencyConfig().pitchRollHz(), 1e-9);
```

- [ ] Add `DriveSimulationPoseTrackerTest` around a package-private wrapper backed by `SwerveDriveOdometry`. Feed known module positions/gyro angles and assert movement; reset estimator-side test state without calling tracker reset and assert truth is unchanged; call tracker reset and assert truth moves to the requested pose.

Lock the wrapper API:

```java
final class DriveSimulationPoseTracker {
  DriveSimulationPoseTracker(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPose);
  void update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions);
  void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose);
  Pose2d getPose();
}
```

- [ ] Add `DriveVisionSupportTest` for the locked public API:

```java
Optional<Pose2d> samplePoseAt(double timestampSeconds);
double getPitchDegrees();
double getRollDegrees();
boolean isPitchRollStableForVision(double maxAbsTiltDegrees);
Pose2d getSimulationPose();
void setPose(Pose2d pose);                    // estimator only
void setPoseAndSimulationTruth(Pose2d pose);  // estimator plus SIM truth
```

Test inclusive tilt equality, positive/negative pitch and roll, present/missing timestamp samples, and distinct reset semantics. Instantiate at most one `Drive` in this test class to avoid repeated process-global AutoBuilder configuration; use the tracker directly for remaining cases.

- [ ] Run the red gate:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.drive.DriveSimulationPoseTrackerTest" --tests "frc.robot.subsystems.drive.DriveVisionSupportTest" --tests "frc.robot.subsystems.drive.GyroVisionTelemetryTest"
```

Expected: compilation fails because the new inputs, tracker, and Drive methods do not exist.

- [ ] Add `pitchDegrees` and `rollDegrees` to `GyroIOInputs`. In Pigeon IO, acquire pitch and roll signals, set both to `50 Hz`, then call `optimizeBusUtilization()`. Include both in the same `refreshAll` call that establishes connection status, and only call `getValueAsDouble()` after that refresh. In NavX IO, publish `navX.getPitch()` and `navX.getRoll()` with the target's existing sign conventions documented in the method.

- [ ] Implement `DriveSimulationPoseTracker` and update it from every odometry sample using the same `rawGyroRotation` and module positions sent to `poseEstimator.updateWithTime`. Do not add vision measurements to this tracker.

- [ ] In `Drive`, delegate `samplePoseAt` directly to `poseEstimator.sampleAt(timestampSeconds)`. Keep `setPose` estimator-only. Make `setPoseAndSimulationTruth` call `setPose` and reset the tracker only when `Constants.currentMode == Mode.SIM`. Change AutoBuilder's reset callback from `this::setPose` to `this::setPoseAndSimulationTruth`. Keep the existing driver Start+Back call on `setPose`.

- [ ] Rerun the focused gate. Expected: `BUILD SUCCESSFUL`; grep confirms no `fpgaToCurrentTime` in the target source.

```powershell
rg -n "fpgaToCurrentTime|Utils\." src/main/java/frc/robot
```

Expected grep result: no vision or Drive timestamp conversion.

- [ ] Commit only Task 5:

```powershell
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' add src/main/java/frc/robot/subsystems/drive/DriveSimulationPoseTracker.java src/main/java/frc/robot/subsystems/drive/GyroIO.java src/main/java/frc/robot/subsystems/drive/GyroIOPigeon2.java src/main/java/frc/robot/subsystems/drive/GyroIONavX.java src/main/java/frc/robot/subsystems/drive/Drive.java src/test/java/frc/robot/subsystems/drive/DriveSimulationPoseTrackerTest.java src/test/java/frc/robot/subsystems/drive/DriveVisionSupportTest.java src/test/java/frc/robot/subsystems/drive/GyroVisionTelemetryTest.java
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' commit -m "feat(drive): add timestamped vision and simulation truth support"
```

---

### Task 6: Build AdvantageKit orchestration, filtering, consensus, and replay

**Files:**

- Create: `src/main/java/frc/robot/subsystems/vision/VisionDriveBindings.java`
- Create: `src/main/java/frc/robot/subsystems/vision/Vision.java`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionTest.java`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionLoggingTest.java`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionReplayTest.java`

**Interfaces:**

- Consumes: the IO, filter, consensus, and Drive APIs produced by Tasks 1–5 plus existing AdvantageKit `Logger` and WPILib `Alert`.
- Produces: `VisionDriveBindings`, the public and injected `Vision` constructors, `Vision.setAiming`, camera-specific logging, replay-safe periodic processing, and the single weighted-measurement handoff used by Tasks 7–10.

- [ ] Add the callback record and fake it in tests:

```java
@FunctionalInterface
interface VisionMeasurementConsumer {
  void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations);
}

record VisionDriveBindings(
    VisionMeasurementConsumer measurementConsumer,
    Consumer<Pose2d> estimatorReset,
    Supplier<Pose2d> currentPose,
    DoubleFunction<Optional<Pose2d>> timestampedPose,
    Supplier<ChassisSpeeds> chassisSpeeds,
    DoubleSupplier pitchDegrees,
    DoubleSupplier rollDegrees) {
  static VisionDriveBindings fromDrive(Drive drive);
}
```

The adapter passes the timestamp to `Drive.samplePoseAt` and `Drive.addVisionMeasurement` unchanged.

- [ ] Add `VisionTest` using scripted fake IO, a mutable FPGA clock, a mutable disabled supplier, and recording drive callbacks. Lock constructors:

```java
public Vision(
    VisionDriveBindings drive,
    VisionRuntimeConfig config,
    Runnable beforeCameraInputs,
    VisionIO... io);

Vision(
    VisionDriveBindings drive,
    VisionRuntimeConfig config,
    BooleanSupplier disabled,
    DoubleSupplier nowSeconds,
    Runnable beforeCameraInputs,
    boolean releaseStartupStrategyOnInitialization,
    VisionIO... io);
```

Production uses `DriverStation::isDisabled`, `Timer::getFPGATimestamp`, and `releaseStartupStrategyOnInitialization=true`. The package-private boolean exists solely so Task 9 can hold both candidates in startup strategy while still exercising initialization state and logging.

- [ ] Test the exact per-loop flow:

  1. Capture `nowSeconds` once.
  2. Run `beforeCameraInputs` once.
  3. Update and `Logger.processInputs("Vision/<fixed IO name>", inputs)` for every camera.
  4. Validate every observation timestamp before requesting history.
  5. Apply hard gates and covariance.
  6. Apply temporal selection, then spatial selection.
  7. Call the measurement consumer at most once with the selected real pose, timestamp, and covariance.

Use a clock supplier that increments on every invocation and assert it is called exactly once. Use a history supplier that throws if an invalid timestamp reaches it and assert invalid timestamp inputs make zero history calls. If history is missing for a valid timestamp, use `drive.currentPose()` only as the innovation reference; heading-seeded Photon strategies remain unavailable for that frame. Assert one accepted candidate preserves its exact FPGA timestamp and covariance. Assert `setAiming(true)` changes translation uncertainty by the `0.6` factor and defaults false.

- [ ] Add multi-camera cases: four coherent candidates produce one consumer call; two clusters pick the approved temporal winner before spatial consensus; a one-camera backlog contributes one newest vote; a spatial loser is logged but never sent; no candidates produce no call.

- [ ] Add `VisionReplayTest`. Instantiate four named `VisionIO.NoOp` streams, populate generated inputs through `LogTable`, and assert fixed configured names choose `Vision/<camera>` process-input keys before replayed `cameraName` is restored. Verify replay constructs no `PhotonCamera`, `PhotonCameraSim`, or `VisionSystemSim`.

- [ ] Add `VisionLoggingTest` with the repository's Logger reflection harness. Assert every required key is written and then cleared on a following empty loop. The key contract is:

```text
Vision/<camera>/RawRobotPoses
Vision/<camera>/AcceptedRobotPoses
Vision/<camera>/RejectedRobotPoses
Vision/<camera>/SupersededRobotPoses
Vision/<camera>/SkewRejectedRobotPoses
Vision/<camera>/SelectedRobotPoses
Vision/<camera>/TagPoses
Vision/<camera>/TagIDs
Vision/<camera>/ActiveStrategy
Vision/<camera>/ObservationType
Vision/<camera>/RejectionReason
Vision/<camera>/ConfidenceDistanceMeters
Vision/<camera>/Ambiguity
Vision/<camera>/CaptureTimestampSeconds
Vision/<camera>/TimestampSkewSeconds
Vision/<camera>/PitchDegrees
Vision/<camera>/RollDegrees
Vision/<camera>/InnovationMeters
Vision/<camera>/InnovationBypassedInDisabledMeters
Vision/RawRobotPoses
Vision/AcceptedRobotPoses
Vision/RejectedRobotPoses
Vision/SelectedRobotPoses
Vision/Consensus/SelectedCamera
Vision/Consensus/CandidateCount
Vision/Consensus/SelectedClusterSize
Vision/Consensus/SelectedStdDevMeters
Vision/Consensus/SelectedInnovationMeters
Vision/Consensus/SelectedTimestampSeconds
Vision/Aiming
Vision/LinearSpeedMetersPerSecond
Vision/AngularSpeedRadiansPerSecond
Vision/AcceptedFieldPoses
Vision/AcceptedTagToPoseLines
Vision/LoopExecutionMilliseconds
Vision/Config/StrategyMode
Vision/Config/StartupStrategyOrder
Vision/Config/ExplicitStrategyOrder
Vision/Config/TagDistanceConfidenceMode
```

Connection state also drives one warning `Alert` per fixed camera name. Test one disconnected camera does not stop healthy cameras.

- [ ] Run the red gate:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.vision.VisionTest" --tests "frc.robot.subsystems.vision.VisionLoggingTest" --tests "frc.robot.subsystems.vision.VisionReplayTest"
```

Expected: compilation fails because `VisionDriveBindings` and `Vision` do not exist.

- [ ] Implement `Vision` by adapting the reference subsystem into the target data flow. Use `Logger.processInputs` for inputs, `Logger.recordOutput` for output keys, `Alert` for camera connection, `VisionFilter` for gates/covariance, and `VisionConsensus` for both selection stages. Unknown tag IDs are skipped when building tag overlays.

- [ ] Rerun the focused gate. Expected: `BUILD SUCCESSFUL`, one measurement per loop, and all arrays/strings empty after the clearing loop.

- [ ] Commit only Task 6:

```powershell
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' add src/main/java/frc/robot/subsystems/vision/VisionDriveBindings.java src/main/java/frc/robot/subsystems/vision/Vision.java src/test/java/frc/robot/subsystems/vision/VisionTest.java src/test/java/frc/robot/subsystems/vision/VisionLoggingTest.java src/test/java/frc/robot/subsystems/vision/VisionReplayTest.java
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' commit -m "feat(vision): fuse coherent observations with AdvantageKit logging"
```

---

### Task 7: Add initialization, accepted snapshots, and estimator-only reseeding

**Files:**

- Modify: `src/main/java/frc/robot/subsystems/vision/Vision.java`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionInitializationReseedTest.java`

**Interfaces:**

- Consumes: `Vision` periodic winner selection and the `VisionIO.markVisionInitializationComplete`/estimator-reset callbacks from Tasks 1 and 6.
- Produces: `Vision.AcceptedObservationSnapshot`, `Vision.getLatestAcceptedObservationSnapshot`, `Vision.forceReseedFromVision`, per-camera initialization state, and disabled automatic-reseed behavior used by Task 10.

- [ ] Add tests for a package-private per-camera initialization state. Only the consensus-selected `PHOTONVISION_MULTITAG_COPROCESSOR` observation advances its camera. Timestamps must strictly increase; translation delta `<= 0.20 m` and heading delta `<= 10 degrees` are stable; an unstable step restarts at one. Another camera or solver does not clear a camera's streak. Five stable observations complete global initialization.

- [ ] Assert completion calls every IO's `markVisionInitializationComplete()` once when `releaseStartupStrategyOnInitialization=true`, and calls none when false. The false seam must not suppress streak state, completion logging, snapshots, or fusion.

- [ ] Lock the snapshot API and defensive-copy behavior:

```java
public record AcceptedObservationSnapshot(Pose2d pose, int[] tagIds, double timestampSeconds) {
  public AcceptedObservationSnapshot { tagIds = Arrays.copyOf(tagIds, tagIds.length); }
  @Override public int[] tagIds() { return Arrays.copyOf(tagIds, tagIds.length); }
}

public Optional<AcceptedObservationSnapshot> getLatestAcceptedObservationSnapshot();
public boolean forceReseedFromVision();
```

Assert only a strictly newer selected timestamp replaces the snapshot. Equal/out-of-order timestamps do not. At age exactly `0.5 s`, the snapshot is available; just older is unavailable.

- [ ] Assert disabled snapshot behavior. While disabled, only selected coprocessor-multitag observations update the snapshot. Preserve the reference transition edge: a recent enabled snapshot with at least two IDs remains eligible for the first disabled automatic reset until it expires or a newer disabled coprocessor-multitag snapshot replaces it.

- [ ] Assert automatic reseed rules with recording estimator-reset and truth-reset callbacks:

  - Never reset while enabled.
  - While disabled, require a recent snapshot and at least two IDs.
  - Reset once initially in each disabled period.
  - Reset again only when translation drift is greater than `0.25 m` and at least `0.5 s` elapsed.
  - Equality at the drift threshold does not trigger because the rule is “more than.” Equality at the interval is allowed.
  - Leaving disabled clears the initial-reset flag.
  - A successful disabled auto-reseed marks initialization complete.
  - Estimator reset count changes; simulation truth reset count remains zero.

- [ ] Assert manual behavior. `forceReseedFromVision()` succeeds from any recent accepted snapshot, including single-tag and enabled snapshots; returns false for stale/missing snapshots; does not mark initialization complete; and calls only the estimator reset.

- [ ] Extend the logger assertions for:

```text
Vision/Initialization/<camera>/StablePoseCount
Vision/Initialization/Complete
Vision/Snapshot/AgeSeconds
Vision/Snapshot/TagIDs
Vision/Reseed/AutomaticSucceeded
Vision/Reseed/ManualSucceeded
Vision/Reseed/Pose
Vision/Reseed/DeltaMeters
Vision/Reseed/TimestampSeconds
Vision/Reseed/RejectionReason
```

- [ ] Run the red gate:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.vision.VisionInitializationReseedTest" --tests "frc.robot.subsystems.vision.VisionLoggingTest"
```

Expected: tests compile against the Task 6 API and fail assertions because initialization, snapshot, and reseed state are absent.

- [ ] Port the reference state machine exactly, preserving per-camera streaks and the enabled-to-disabled snapshot edge. Invoke automatic reseed after consensus/snapshot processing in each periodic loop. Do not reset the independent truth callback anywhere in `Vision`.

- [ ] Rerun the focused gate. Expected: `BUILD SUCCESSFUL` for every stability boundary and reseed rule.

- [ ] Commit only Task 7:

```powershell
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' add src/main/java/frc/robot/subsystems/vision/Vision.java src/test/java/frc/robot/subsystems/vision/VisionInitializationReseedTest.java src/test/java/frc/robot/subsystems/vision/VisionLoggingTest.java
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' commit -m "feat(vision): add initialization and vision reseeding"
```

---

### Task 8: Add one shared Photon simulation and isolate JNI/global-state tests

**Files:**

- Modify: `build.gradle`
- Modify: `src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java`
- Create: `src/main/java/frc/robot/subsystems/vision/VisionSimulation.java`
- Create: `src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVisionSim.java`
- Create: `src/test/java/frc/robot/subsystems/vision/VisionSimulationHarness.java`
- Create: `src/test/java/frc/robot/subsystems/vision/sim/VisionSimulationLifecycleTest.java`
- Create: `src/test/java/frc/robot/subsystems/vision/sim/VisionPhotonScenariosTest.java`
- Modify: `docs/superpowers/specs/2026-07-31-vision-subsystem-port-design.md`
- Modify: `docs/superpowers/plans/2026-07-31-vision-subsystem-port.md`

**Interfaces:**

- Consumes: `VisionIOPhotonVision`, `Vision`, `CameraConfig`, `VisionRuntimeConfig`, `VisionDriveBindings`, and independent Drive truth from Tasks 1, 4–7.
- Produces: `VisionSimulation`, `VisionIOPhotonVisionSim`, the `visionSimulationTest` Gradle task, and `VisionSimulationHarness` metrics/sampling hooks used by Tasks 9–10.

- [ ] Add `@Tag("vision-sim")` to both simulation test classes. Add the tests before defining the Gradle task, then run:

```powershell
.\gradlew.bat visionSimulationTest --tests "frc.robot.subsystems.vision.sim.VisionSimulationLifecycleTest"
```

Expected: Gradle fails with `Task 'visionSimulationTest' not found`.

- [ ] Configure normal and isolated tests exactly once in `build.gradle`:

```groovy
test {
    useJUnitPlatform {
        excludeTags 'vision-sim'
    }
    systemProperty 'junit.jupiter.extensions.autodetection.enabled', 'true'
}

def visionSimulationTest = tasks.register('visionSimulationTest', Test) {
    group = 'verification'
    description = 'Runs isolated PhotonVision JNI and global-state scenarios.'
    testClassesDirs = sourceSets.test.output.classesDirs
    classpath = sourceSets.test.runtimeClasspath
    useJUnitPlatform {
        includeTags 'vision-sim'
    }
    systemProperty 'junit.jupiter.extensions.autodetection.enabled', 'true'
    forkEvery = 1
    maxParallelForks = 1
}

wpi.java.configureTestTasks(visionSimulationTest.get())
check.dependsOn(visionSimulationTest)
```

Retain the existing `wpi.java.configureTestTasks(test)` call. Do not use class-name filters as the primary split; tags define the contract.

- [ ] Lock `VisionSimulation` ownership:

```java
public final class VisionSimulation implements AutoCloseable {
  public VisionSimulation(Supplier<Pose2d> poseSupplier);
  public VisionSimulation(String instanceName, Supplier<Pose2d> poseSupplier);
  void registerCamera(PhotonCamera camera, CameraConfig config);
  public void update();
  public void setPoseSupplier(Supplier<Pose2d> poseSupplier);
  public int cameraCount();
  public long updateCount();
  public List<CameraDiagnostics> cameraDiagnostics();
}
```

The production instance name is `main`; tests supply a unique name through the public instance-name constructor. The owner adds `FieldConstants.APTAG_FIELD_LAYOUT` once. It stores one pose supplier, four camera simulations, and closes every camera simulation and camera. `VisionIOPhotonVision` adds a package-private constructor accepting an already-created `PhotonCamera` with the camera/runtime configuration and heading provider while leaving its public REAL constructor unchanged. `VisionIOPhotonVisionSim(CameraConfig, VisionRuntimeConfig, HeadingProvider, VisionSimulation)` creates one `PhotonCamera`, passes that exact camera to the real decoder constructor, registers the same instance with the owner, and never calls `VisionSystemSim.update()`.

`CameraDiagnostics` is an immutable public snapshot containing the camera name, approved transform, resolution, geometric diagonal FOV, calibration error, FPS, and latency properties. Values come from the registered `PhotonCameraSim` property getters wherever exposed; calibration-error values and a deterministic seeded pixel-noise sample come from the single settings path that invokes `setCalibError`. PhotonLib 2026.3.4 expands the horizontal/vertical getters to the coordinates at `width`/`height`, one pixel beyond the final valid center, and its `getDiagFOV()` takes a hypot of angles. Derive the `72 degree` geometric diagonal from the actual registered intrinsics using valid `(resolution - 1)` pixel-center extents.

- [ ] In `VisionSimulationLifecycleTest`, construct one owner and all four ordered adapters. Assert four registered camera names/transforms, no duplicate systems, and these exact properties for each camera:

| Property | Value |
|---|---:|
| resolution | `800 x 600` |
| diagonal FOV | `72 degrees` |
| calibration error | `(0.38, 0.1)` |
| frame rate | `60 FPS` |
| average latency | `10 ms` |
| latency standard deviation | `5 ms` |

Call all four IO `updateInputs` methods without calling owner `update`; assert `updateCount()==0`. Call the owner's `update()` once, read all four inputs, and assert `updateCount()==1`. Construct `Vision` with `owner::update` and assert one periodic call increments it by exactly one before the first camera read.

Assert the diagnostics list is immutable, duplicate camera names are rejected, the seeded pixel-noise sample changes if `setCalibError(0.38, 0.1)` is missing or wrong, and the FOV regression fails if `resolution` is used in place of the valid `resolution - 1` pixel-center extent. Assert close clears the public camera count, is idempotent, and prevents later updates.

- [ ] Build `VisionSimulationHarness` around an independent mutable truth pose, `TimeInterpolatableBuffer<Pose2d>` truth history, a separate estimator pose, recording measurement consumer, deterministic `20 ms` loop increments, and one shared owner. Detections must be generated from the truth supplier, never the estimator supplier.

For selected observations, score error against truth sampled at the selected capture timestamp. For coverage, count new measurement-consumer calls or distinct newly selected timestamps; do not count how long a snapshot remains populated.

- [ ] In `VisionPhotonScenariosTest`, run all eight approved stationary poses:

```text
(4.407, 7.279, -90 deg)  (4.407, 7.279, +90 deg)
(4.407, 7.279, 180 deg)  (4.407, 7.279,   0 deg)
(4.407, 0.650, +90 deg)  (4.407, 0.650, -90 deg)
(4.407, 0.650,   0 deg)  (4.407, 0.650, 180 deg)
```

For each pose, warm up `100` cycles (`2 s`) and measure `250` cycles (`5 s`). Require at least one camera detection, at least one accepted consumer call, and finite capture-time error. Add two deterministic moving scenarios: a `0.65 m/s` translation with `0.25 rad/s` heading change and an in-place `0.40 rad/s` rotation. Both rates keep constrained and trig solvers eligible so their startup ordering is exercised.

- [ ] Run the red simulation gate after the Gradle task exists:

```powershell
.\gradlew.bat visionSimulationTest --tests "frc.robot.subsystems.vision.sim.VisionSimulationLifecycleTest" --tests "frc.robot.subsystems.vision.sim.VisionPhotonScenariosTest"
```

Expected: compilation fails because `VisionSimulation`, `VisionIOPhotonVisionSim`, and the harness do not exist.

- [ ] Implement the shared owner, simulated adapter, and harness. Give each isolated test a unique NetworkTables/vision-system name and close resources in `@AfterEach` or try-with-resources. A fixed simulation random seed may be applied, but the approved optical and latency values must remain unchanged.

- [ ] Rerun both gates:

```powershell
.\gradlew.bat test
.\gradlew.bat visionSimulationTest --tests "frc.robot.subsystems.vision.sim.VisionSimulationLifecycleTest" --tests "frc.robot.subsystems.vision.sim.VisionPhotonScenariosTest"
```

Expected: normal tests exclude `vision-sim`; the isolated task reports `BUILD SUCCESSFUL` with nonzero detections and accepted coverage.

- [ ] Commit only Task 8:

```powershell
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' add build.gradle src/main/java/frc/robot/subsystems/vision/VisionSimulation.java src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVisionSim.java src/test/java/frc/robot/subsystems/vision/VisionSimulationHarness.java src/test/java/frc/robot/subsystems/vision/sim/VisionSimulationLifecycleTest.java src/test/java/frc/robot/subsystems/vision/sim/VisionPhotonScenariosTest.java
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' commit -m "test(vision): add isolated four-camera Photon simulation"
```

---

### Task 9: Compare both startup orders and pin the deterministic winner

**Files:**

- Modify: `src/main/java/frc/robot/subsystems/vision/VisionConstants.java`
- Create: `src/test/java/frc/robot/subsystems/vision/VisionStartupStrategyComparisonSupport.java`
- Create: `src/test/java/frc/robot/subsystems/vision/VisionStartupStrategyComparatorTest.java`
- Create: `src/test/java/frc/robot/subsystems/vision/sim/VisionStartupStrategyComparisonTest.java`
- Modify: `src/test/java/frc/robot/subsystems/vision/VisionSimulationHarness.java`
- Modify: `src/test/java/frc/robot/subsystems/vision/VisionConstantsTest.java`

**Interfaces:**

- Consumes: both `StartupStrategyOrder` values, the Task 6 constructor seam that retains startup strategy, and the Task 8 independent-truth harness.
- Produces: public test-support `StrategyMetrics`, `VisionStartupStrategyComparisonSupport.winner`, a normal regression over the last verified raw metrics, and the measured `VisionConstants.DEFAULT_STARTUP_STRATEGY_ORDER` consumed by production runtime configuration.

- [ ] Add a small public test-support comparator in package `frc.robot.subsystems.vision`, not the reference CSV/report framework:

```java
record StrategyMetrics(
    StartupStrategyOrder order,
    double maximumStationaryEstimatorJumpMeters,
    double maximumMovingExcessJumpMeters,
    int movingExcessJumpOverFortyCentimetersCount,
    double observationCoverage,
    int acceptedObservationCount,
    double meanAcceptedPoseErrorMeters,
    double worstAcceptedPoseErrorMeters) {
  boolean passesSafetyGates();
}

static Optional<StartupStrategyOrder> winner(
    StrategyMetrics reference, StrategyMetrics constrainedSecond);
```

Safety gates are exact:

```text
maximum stationary estimator jump <= 0.15 m
maximum moving excess jump <= 0.25 m
moving cycles with excess jump > 0.40 m == 0
coverage >= 0.30
accepted observation count > 0
```

- [ ] Add untagged `VisionStartupStrategyComparatorTest` cases for every branch and tolerance equality:

  1. Only one safe order wins.
  2. Neither safe order returns `Optional.empty()`; the scenario test then fails with both metric records.
  3. If both pass and mean errors differ by more than `0.02 m`, lower mean wins.
  4. Otherwise, if worst errors differ by more than `0.05 m`, lower worst wins.
  5. Otherwise, if coverage differs by more than `0.05` absolute, higher coverage wins.
  6. Otherwise, constrained-second wins.

Equality at each tolerance advances to the next metric.

- [ ] Add `@Tag("vision-sim")` comparison tests. Run both orders through identical seeds, eight stationary poses, and both moving paths from Task 8. Construct `Vision` with `releaseStartupStrategyOnInitialization=false`. This deliberately freezes only the IO startup preference; initialization streaks, completion state, logging, filtering, consensus, snapshots, and estimator fusion continue normally for the full measured window.

- [ ] Define metrics precisely:

  - Stationary estimator jump is the translation distance between consecutive estimator poses while truth is fixed.
  - Moving excess jump is `max(0, estimatorStepDistance - truthStepDistance)`.
  - Coverage is measurement-consumer calls divided by measured scheduler cycles.
  - Accepted-pose error is selected camera pose versus interpolated independent truth at that observation's capture timestamp.
  - Mean/worst error includes every accepted consumer call across all scenarios.

Print both complete metric records to the test output before asserting safety or winner. Do not write files.

- [ ] Run the red gate:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.vision.VisionStartupStrategyComparatorTest"
.\gradlew.bat visionSimulationTest --tests "frc.robot.subsystems.vision.sim.VisionStartupStrategyComparisonTest"
```

Expected: compilation fails because comparison support does not exist; after adding only the pure support, the isolated scenario still fails because the comparison scenario is not implemented.

- [ ] Implement support and run the exact same command. If neither order passes, stop implementation and report the raw metrics; do not weaken gates. If one or both pass, record the computed winner.

- [ ] Set `VisionConstants.DEFAULT_STARTUP_STRATEGY_ORDER` to the computed winner. It begins as `CONSTRAINED_SECOND`, the approved tie preference, and changes only if the measured comparator selects `REFERENCE`. Copy both complete printed `StrategyMetrics` constructor values from that passing run into `VisionStartupStrategyComparatorTest` as `LAST_VERIFIED_REFERENCE_METRICS` and `LAST_VERIFIED_CONSTRAINED_SECOND_METRICS`. Add an untagged assertion that recomputes `winner(...)` from those two raw records and equals the compiled default.

- [ ] Rerun the comparison a second time and confirm identical winner and metrics within floating-point tolerance, then run the normal constant test:

```powershell
.\gradlew.bat visionSimulationTest --tests "frc.robot.subsystems.vision.sim.VisionStartupStrategyComparisonTest"
.\gradlew.bat test --tests "frc.robot.subsystems.vision.VisionConstantsTest" --tests "frc.robot.subsystems.vision.VisionStartupStrategyComparatorTest"
```

Expected: both commands report `BUILD SUCCESSFUL`; the compiled default matches the deterministic winner.

- [ ] Commit only Task 9; retain both measured metric records in the regression test and report the winner in the implementation handoff:

```powershell
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' add src/main/java/frc/robot/subsystems/vision/VisionConstants.java src/test/java/frc/robot/subsystems/vision/VisionStartupStrategyComparisonSupport.java src/test/java/frc/robot/subsystems/vision/VisionStartupStrategyComparatorTest.java src/test/java/frc/robot/subsystems/vision/sim/VisionStartupStrategyComparisonTest.java src/test/java/frc/robot/subsystems/vision/VisionSimulationHarness.java src/test/java/frc/robot/subsystems/vision/VisionConstantsTest.java
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' commit -m "test(vision): select deterministic startup strategy order"
```

---

### Task 10: Integrate mode construction and operator controls

**Files:**

- Create: `src/main/java/frc/robot/VisionConstructionPlan.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Test: `src/test/java/frc/robot/VisionConstructionPlanTest.java`
- Test: `src/test/java/frc/robot/RobotContainerVisionTest.java`
- Test: `src/test/java/frc/robot/RobotContainerVisionBindingTest.java`

**Interfaces:**

- Consumes: all production vision/Drive types from Tasks 1–9, `Constants.Mode`, both controller ports, and the existing driver Start+Back binding.
- Produces: `VisionConstructionPlan.forMode`, deferred per-camera IO factories, the retained `RobotContainer.vision` stack, `RobotContainer.setVisionSimulationPoseSupplier`, and the enabled-only operator Start+Back command.

- [ ] Add pure construction-plan tests before touching `RobotContainer`:

```java
record VisionConstructionPlan(IoKind ioKind, List<CameraConfig> cameras) {
  enum IoKind { REAL_PHOTON, SIM_PHOTON, REPLAY_NOOP }
  static VisionConstructionPlan forMode(Constants.Mode mode);
}
```

Assert REAL, SIM, and REPLAY map to the three IO kinds and each returns an immutable four-element camera list with exact ordered names/transforms. The plan must not construct Photon, CAN, `Drive`, or AutoBuilder objects.

- [ ] Add package-private deferred factories in `RobotContainer`, following its existing Shooter/Intake REAL-hardware pattern. Tests may inspect the REAL implementation class/supplier without calling `create()`. SIM materialization receives one shared `VisionSimulation`; REPLAY materialization returns four named `VisionIO.NoOp` objects.

```java
record VisionIOFactory(
    Class<? extends VisionIO> implementationType,
    Supplier<? extends VisionIO> constructor) {
  VisionIO create() { return constructor.get(); }
}

static List<VisionIOFactory> visionIOFactories(
    VisionConstructionPlan plan,
    VisionRuntimeConfig config,
    VisionIOPhotonVision.HeadingProvider headingProvider,
    VisionSimulation simulation);
```

For REAL and REPLAY, `simulation` is null and never dereferenced. For SIM, every supplier closes over the same non-null owner. `RobotContainer` calls `create()` exactly once for each of the four factories after the Drive and runtime config exist.

- [ ] Add `RobotContainerVisionTest` for:

  - Capturing `VisionRuntimeConfig.fromSystemProperties()` once per container construction.
  - Building `Drive` before the vision stack.
  - REAL factories naming `VisionIOPhotonVision` without constructing them in the test.
  - SIM deferred factories naming four `VisionIOPhotonVisionSim` adapters and retaining the same owner reference without materializing Photon objects in the normal test.
  - REPLAY factories creating four no-ops with fixed camera names.
  - `setAiming(false)` as the startup state.
  - `setVisionSimulationPoseSupplier(Supplier<Pose2d>)` replacing the shared owner's truth supplier in SIM and acting as a safe no-op outside SIM.

- [ ] In production construction, create `VisionDriveBindings.fromDrive(drive)`. Derive a Photon heading provider from timestamped pose rotation, timestamped pose as `Pose3d`, measured angular rate, and measured linear speed. Use `drive::getSimulationPose` as the default shared SIM truth supplier. Pass `visionSimulation::update` as the `Vision` pre-input hook; use an empty hook for REAL/REPLAY.

- [ ] Add the operator command without disabled override:

```java
operatorController
    .start()
    .and(operatorController.back())
    .onTrue(
        Commands.runOnce(vision::forceReseedFromVision, vision)
            .withName("Force Vision Reseed"));
```

Keep the driver Start+Back binding on estimator-only `drive.setPose(...)` and retain `.ignoringDisable(true)`. Naming that existing command `Driver Heading Reset` is allowed for test observability but must not change its controller, pose calculation, subsystem requirement, or disabled behavior.

- [ ] Add `@Tag("vision-sim")` to `RobotContainerVisionBindingTest` so it runs in a fresh fork. Initialize HAL once, instantiate one SIM `RobotContainer`, use `XboxControllerSim` and `DriverStationSim`, and register a `CommandScheduler.onCommandInitialize` listener. Assert:

  - Enabled operator Start+Back initializes `Force Vision Reseed`.
  - Disabled operator Start+Back does not initialize it.
  - Disabled driver Start+Back still initializes `Driver Heading Reset`.
  - The operator and driver use their distinct configured ports.

- [ ] Run the red gates:

```powershell
.\gradlew.bat test --tests "frc.robot.VisionConstructionPlanTest" --tests "frc.robot.RobotContainerVisionTest"
.\gradlew.bat visionSimulationTest --tests "frc.robot.RobotContainerVisionBindingTest"
```

Expected: compilation fails because the construction plan and container integration do not exist.

- [ ] Implement the plan/factories, retain `private final Vision vision`, retain a nullable/optional SIM owner only for pose-supplier overrides, and add both bindings. Do not create temporary Superstructure calls; aiming remains false until the future Superstructure owns it.

- [ ] Rerun both focused gates. Expected: `BUILD SUCCESSFUL`, no REAL hardware constructors invoked in normal tests, and the enabled-only operator behavior is pinned.

- [ ] Commit only Task 10:

```powershell
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' add src/main/java/frc/robot/VisionConstructionPlan.java src/main/java/frc/robot/RobotContainer.java src/test/java/frc/robot/VisionConstructionPlanTest.java src/test/java/frc/robot/RobotContainerVisionTest.java src/test/java/frc/robot/RobotContainerVisionBindingTest.java
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' commit -m "feat(robot): integrate four-camera vision by runtime mode"
```

---

### Task 11: Run the complete gate, inspect the exact diff, and request review

**Files:** All files changed by Tasks 1–10; no new production scope.

**Interfaces:**

- Consumes: every implementation, focused test, commit, and metric produced by Tasks 1–10 plus the approved design document.
- Produces: fresh full-suite evidence, exact diff hygiene, an independent review result, and a publication-ready local branch without pushing it.

- [ ] Invoke `superpowers:verification-before-completion` before running the final gate.

- [ ] Verify dependency resolution and compilation:

```powershell
.\gradlew.bat testClasses
```

Expected: `BUILD SUCCESSFUL`; generated `VisionIOInputsAutoLogged` exists.

- [ ] Run the complete normal suite:

```powershell
.\gradlew.bat test
```

Expected: `BUILD SUCCESSFUL`; the report contains no `vision-sim` tagged tests.

- [ ] Run every isolated Photon/global-state scenario:

```powershell
.\gradlew.bat visionSimulationTest
```

Expected: `BUILD SUCCESSFUL`; lifecycle, eight stationary poses, moving scenarios, comparator, and binding test all run in isolated forks. Record both strategy metric sets and the winning order in the handoff.

- [ ] Run the complete verification lifecycle and formatting:

```powershell
.\gradlew.bat check
.\gradlew.bat spotlessCheck
```

Expected: both commands report `BUILD SUCCESSFUL`; `check` invokes `visionSimulationTest`.

- [ ] Prove the two approved fixes and prohibited coupling with focused searches:

```powershell
rg -n "new Translation3d\(1\.0, 0\.0, 0\.0\)|MAX_CONSENSUS_TIMESTAMP_SPAN_SECONDS|SUPERSEDED_BY_NEWER_CAMERA_FRAME|TIMESTAMP_CLUSTER_NOT_SELECTED" src/main/java src/test/java
rg -n "fpgaToCurrentTime|DogLog|lombok|static\s+.*VisionSystemSim" src/main/java src/test/java
```

Expected: the first search finds implementation and regression tests; the second finds no prohibited usage.

- [ ] Review exact branch scope and hygiene:

```powershell
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' status --short --branch
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' diff main...HEAD --stat
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' diff --check main...HEAD
git -c safe.directory='C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_AK_Swerve_SPF2_Robot' log --oneline --decorate main..HEAD
```

Expected: clean worktree, only vision/drive integration plus approved docs/dependency/build files, no whitespace errors, and one semantic commit per task.

- [ ] Perform a requirement audit against `docs/superpowers/specs/2026-07-31-vision-subsystem-port-design.md`. Check each item explicitly:

  - Four exact REAL/SIM/REPLAY cameras.
  - All unread results and full solver chains.
  - Runtime property capture and both distance policies.
  - Every hard gate and covariance factor.
  - Timestamp validity before history lookup.
  - Newest-per-camera, 40 ms temporal windows, and spatial consensus.
  - Positive-X tag normals.
  - Five-stable-multitag initialization.
  - Snapshot freshness, disabled auto-reseed, and enabled-only manual command.
  - Pitch/roll refresh and direct FPGA timebase.
  - Independent truth and shared update-once simulation.
  - Camera-specific AdvantageKit inputs/outputs with stale clearing.
  - Deterministic strategy comparator and compiled winner.

- [ ] Invoke `superpowers:requesting-code-review` and assign an independent reviewer the exact `main...HEAD` diff plus the design document. Require the reviewer to report actionable findings with file/line evidence and to distinguish code defects from expected Photon simulation noise.

- [ ] If review finds an issue, invoke `superpowers:receiving-code-review`, reproduce it with a failing focused test, fix only the confirmed defect, rerun the focused gate plus all Task 11 commands, and commit with a narrowly scoped message.

- [ ] Do not push or open a PR in this plan unless the user separately authorizes publication. Report final commit list, test commands/results, strategy metrics/winner, and any expected simulation diagnostics.
