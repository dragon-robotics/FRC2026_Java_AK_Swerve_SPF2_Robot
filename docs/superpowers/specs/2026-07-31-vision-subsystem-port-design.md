# Vision Subsystem Port Design

## Goal

Port the complete production AprilTag vision subsystem from the `spitfire-v2` reference into this
repository's AdvantageKit architecture. Preserve the four-camera PhotonVision solver, filtering,
confidence, consensus, initialization, and reseed behavior while correcting timestamp-incoherent
consensus and incorrect tag-face coplanarity detection.

The reference baseline is commit `70cce7cc0ee2c3b53644a582acf5b6bb9be35dd8` from
`FRC2026_Java_Swerve_Robot:spitfire-v2`.

## Approved Decisions

- Port the full production subsystem, not only basic pose fusion.
- Keep all four reference camera names and CAD transforms unchanged.
- Adapt the implementation directly to AdvantageKit rather than retaining DogLog, Lombok, or the
  CTRE drivetrain type.
- Add timestamp-coherent cross-camera consensus.
- Correct coplanarity calculations to use the AprilTag face-normal axis.
- Compare two startup strategy orders using an exact metric comparator. If both are safe and fall
  inside every approved tie tolerance, prefer constrained SolvePnP before distance-trig solving.
- Bind operator Start+Back to manual vision reseeding.
- Leave aiming false until the future Superstructure calls `setAiming()`.
- Use an independent pose for PhotonVision simulation rather than the vision-corrected estimator.

## Scope

Included:

- Four PhotonVision AprilTag cameras in REAL mode.
- Four simulated PhotonVision cameras using the same identities and transforms.
- Replay-safe no-op IO for all four camera streams.
- All-unread-frame processing and the complete Photon solver strategy chain.
- Geometric, ambiguity, distance, tilt, and timestamped innovation rejection gates.
- Dynamic covariance, aiming trust, and gyro-authoritative heading.
- Timestamp-coherent, one-observation-per-camera spatial consensus.
- Stable multi-tag initialization tracking.
- Accepted-observation snapshots, disabled automatic reseeding, and manual reseeding.
- AdvantageKit inputs, outputs, alerts, and AdvantageScope field overlays.
- Timestamped drive-pose sampling, pitch/roll telemetry, and independent simulation odometry.
- Unit, integration, replay, and four-camera Photon simulation tests.

Excluded:

- Superstructure behavior beyond exposing `setAiming()`.
- Object detection or game-piece tracking.
- MegaTag producers; the observation enum values remain for compatibility only.
- Automatic pose resets while enabled.
- Vision heading fusion during normal operation.
- Camera calibration or CAD-transform retuning.
- Runtime solver research reports, CSV generation, and the reference's count-only bakeoff suite.
- A hardware conclusion based only on simulation; real-camera validation remains a follow-up after
  the deterministic verification gate.

## Architecture

The target-native data path is:

```text
4 Photon cameras
      |
      v
VisionIOPhotonVision / VisionIOPhotonVisionSim
      |  timestamped observations
      v
Vision
      |-- filtering + timestamp coherence + consensus --> one weighted measurement --> Drive
      |-- accepted snapshot ---------------------------> logging and public API
      `-- disabled/manual reseed ----------------------> Drive estimator reset
```

Ownership boundaries:

- `VisionConstants` owns camera identities, CAD transforms, solver thresholds, filtering
  thresholds, confidence parameters, consensus parameters, initialization values, simulation
  properties, and reseed values.
- `VisionRuntimeConfig` captures property-backed settings once at construction and exposes pure
  parsing helpers plus direct test injection.
- `VisionIO` defines one replayable camera snapshot plus immutable target and pose observations.
- `VisionIOPhotonVision` owns `PhotonCamera`, `PhotonPoseEstimator`, result decoding, solver-order
  selection, and pose-attempt execution.
- `VisionIOPhotonVisionSim` adds one `PhotonCameraSim` to a shared, instance-owned simulation
  context.
- `VisionSimulation` owns one `VisionSystemSim`, the field targets, update-once coordination, and
  the independent pose supplier.
- `Vision` owns hard gates, dynamic confidence, timestamp coherence, cross-camera consensus,
  initialization, snapshots, reseeding, alerts, and logging.
- `Drive` owns odometry, the pose estimator, timestamped pose history, gyro telemetry, and an
  independent simulation pose.
- `RobotContainer` realizes a pure REAL, SIM, or REPLAY vision construction plan and owns the
  operator reseed binding.

The target class is named `Vision`, matching this repository's subsystem naming and AdvantageKit
conventions. The behavior remains traceable to the reference `VisionSubsystem`.

## Camera Hardware

All transforms are robot-to-camera transforms in WPILib's robot coordinate frame.

| Camera | Translation inches `(X, Y, Z)` | Roll, pitch, yaw degrees |
|---|---:|---:|
| `AprilTagPoseEstCameraF` | `(-11.152, -7.579, 20.930)` | `(0, -15, 0)` |
| `AprilTagPoseEstCameraR` | `(-8.387, -13.355, 15.931)` | `(0, -12, -90)` |
| `AprilTagPoseEstCameraB` | `(-9.164, 12.500, 20.839)` | `(0, -15, 180)` |
| `AprilTagPoseEstCameraL` | `(-8.387, 13.355, 15.931)` | `(0, -12, 90)` |

The existing `FieldConstants.APTAG_FIELD_LAYOUT` remains the only AprilTag-layout source. It loads
`src/main/deploy/apriltags/welded/2026-rebuilt-welded-no-single.json` and retains the existing
WPILib fallback behavior.

## Dependency

Add the reference PhotonLib vendordep at version `v2026.3.4`. Desktop native dependencies remain
enabled so deterministic Photon solver and camera simulation tests can run through GradleRIO.

No DogLog or Lombok dependency is added. All generated logging support comes from AdvantageKit's
existing annotation processor.

## Vision IO Contract

`VisionIOInputs` is annotated with `@AutoLog` and records:

- Camera name.
- Camera connection state.
- Latest best-target yaw and pitch.
- All pose observations decoded during the update.
- Deduplicated IDs of tags contributing to successfully emitted observations during the update.

Every `VisionIO` also exposes an immutable configured camera name outside the replayed input
snapshot. `Vision` uses that fixed identity to select `Logger.processInputs()` keys before replay
has restored the recorded `cameraName`. `VisionIO.NoOp` therefore requires a camera name.

Each immutable `PoseObservation` records:

- FPGA capture timestamp in seconds.
- Field-relative robot `Pose3d`.
- Mean nonnegative pose ambiguity.
- Positive-fiducial tag count.
- Confidence distance.
- Observation type.
- Successful solver strategy as a target-owned `PoseSolveStrategy` enum.
- A defensively copied array of contributing tag IDs.

Observation types remain:

- `MEGATAG_1`
- `MEGATAG_2`
- `PHOTONVISION`
- `PHOTONVISION_MULTITAG_COPROCESSOR`

The Photon implementation produces only the two PhotonVision types. Multi-tag coprocessor results
are identified explicitly because initialization and disabled snapshots apply stricter rules to
them.

`PoseSolveStrategy` contains the four supported Photon strategies plus `UNKNOWN`. Storing it on
each observation preserves the successful strategy for multiple unread frames and allows
AdvantageKit replay to reproduce strategy telemetry without logging directly from hardware IO.

Every `updateInputs()` call overwrites every field. Connection state does not short-circuit unread
result processing: a disconnected camera still drains any buffered results, matching the reference.
When no unread results exist, the camera publishes its name and connection state, zero target
angles, and empty observation/tag arrays. No previous-frame data remains implicitly active.

## Photon Result Processing

Each camera owns one `PhotonPoseEstimator` using the shared field layout and its approved transform.
Every unread result is processed in capture order.

For a result with targets:

1. Record its best-target yaw and pitch.
2. Skip pose solving when every target's best camera-to-target transform is farther than `8.0 m`.
3. Resolve the strategy order for the current initialization and motion state.
4. Try strategies in order and stop at the first successful pose.
5. Collect valid positive fiducial IDs, ambiguity samples, and distance samples.
6. Emit one observation with the solver's original capture timestamp, successful strategy, and
   contributing tag IDs.

Ambiguity is the mean of `max(0, ambiguity)` across contributing targets. The default confidence
distance is the average across all contributing tags. System property
`vision.tagDistanceConfidenceMode=MAX_TAG_DISTANCE` selects the farthest contributing tag instead.
An individual target whose best transform is missing contributes no distance sample. Positive
fiducial IDs and ambiguity still count exactly as in the reference. If no usable distance sample
remains across the emitted observation, confidence distance becomes positive infinity and is
rejected later. An invalid property value falls back to the documented all-tag average. The
selected runtime mode is logged.

## Runtime Configuration

`VisionRuntimeConfig` reads the reference-compatible JVM properties once when the vision stack is
constructed:

- `vision.maxAbsTiltDeg`, default `8.0`.
- `vision.applyCoplanarPenalty`, default `true`.
- `vision.photon.strategyMode`, default `HYBRID`.
- `vision.photon.strategyOrder`.
- `vision.tagDistanceConfidenceMode`, default `ALL_TAG_AVERAGE`.

This preserves the reference's startup-time property behavior while avoiding static class-load
contamination in tests. Production uses `VisionRuntimeConfig.fromSystemProperties()`. Tests inject
an immutable config directly and exercise the parsing helpers without mutating process-global
properties.

## Solver Strategies

Before initialization, verification compares these complete fallback chains:

1. Reference order:
   `MULTI_TAG_PNP_ON_COPROCESSOR`, `PNP_DISTANCE_TRIG_SOLVE`,
   `CONSTRAINED_SOLVEPNP`, `LOWEST_AMBIGUITY`.
2. Preferred order:
   `MULTI_TAG_PNP_ON_COPROCESSOR`, `CONSTRAINED_SOLVEPNP`,
   `PNP_DISTANCE_TRIG_SOLVE`, `LOWEST_AMBIGUITY`.

The implementation represents these as `StartupStrategyOrder.REFERENCE` and
`StartupStrategyOrder.CONSTRAINED_SECOND`. Tests inject the enum directly, without JVM properties.
The deterministic comparison described under Testing selects the compiled
`DEFAULT_STARTUP_STRATEGY_ORDER` constant. A regression test reruns the comparator and requires the
compiled default to match its winner.

After initialization:

- An explicit nonblank `vision.photon.strategyOrder` property is honored after parsing supported
  strategy names.
- Otherwise `vision.photon.strategyMode=HYBRID` preserves the reference selection rules.
- A non-HYBRID strategy mode uses the static-chain path. Its default chain remains
  `CONSTRAINED_SOLVEPNP`, `MULTI_TAG_PNP_ON_COPROCESSOR`, `PNP_DISTANCE_TRIG_SOLVE`,
  `LOWEST_AMBIGUITY`.
- If every explicit strategy token is invalid, parsing falls back to
  `MULTI_TAG_PNP_ON_COPROCESSOR`, `CONSTRAINED_SOLVEPNP`, `PNP_DISTANCE_TRIG_SOLVE`,
  `LOWEST_AMBIGUITY`.
- Above `0.5 rad/s`, constrained SolvePnP is omitted.
- Above `1.0 rad/s`, distance-trig attempts reject themselves because latency-aligned heading is no
  longer trusted.
- Multi-tag, corrected-coplanar observations prefer constrained SolvePnP when angular rate permits.
- Multi-tag, noncoplanar observations prefer coprocessor multi-tag solving.
- Single-tag observations prefer distance-trig; translation above `0.5 m/s` changes the remaining
  fallback order exactly as in the reference.
- `LOWEST_AMBIGUITY` remains the terminal fallback.

Distance-trig requires a drivetrain heading sampled at the frame timestamp. Constrained SolvePnP
requires the same heading, camera calibration and distortion data, and a seed pose. It first tries
the result's lowest-ambiguity pose as the seed and then falls back to the drivetrain pose sampled at
the capture timestamp. The constrained heading-scale factor remains `0.2`.

## Acceptance Gates

An observation is rejected when any condition is true:

- Tag count is zero.
- Pose `|Z|` exceeds `0.5 m`.
- Pose X or Y lies outside the active field layout.
- A single-tag observation has ambiguity above `0.2`.
- Confidence distance exceeds `5.5 m`. Missing samples arrive as positive infinity and therefore
  fail this reference gate.
- Current chassis pitch or roll exceeds the property-backed maximum, defaulting to `8 degrees` in
  magnitude.
- While enabled, translation innovation exceeds `2.5 m` from the drivetrain pose sampled at the
  observation timestamp.

The innovation gate is bypassed while disabled so a substantially wrong estimator can recover.
The bypass distance is logged. Threshold equality is accepted unless a condition explicitly says
otherwise.

Normal operation never resets pose. Passing observations proceed to consensus and then enter the
WPILib pose estimator as weighted measurements.

## Timestamp-Coherent Consensus Fix

The reference puts every passing unread frame into one spatial pool. A camera backlog can therefore
outvote other cameras, and poses captured at different robot positions can be compared as though
they were simultaneous.

The corrected flow is:

1. Process and log every unread frame.
2. Reject a nonfinite timestamp as `INVALID_TIMESTAMP`.
3. Using one captured FPGA `now` value for the loop, reject a timestamp more than `0.020 s` in the
   future as `FUTURE_TIMESTAMP` or more than `0.500 s` old as `STALE_TIMESTAMP`. Equality at either
   limit is accepted.
4. Retain only the newest passing candidate from each camera for temporal consensus. Older passing
   candidates are logged as `SUPERSEDED_BY_NEWER_CAMERA_FRAME`. Equal timestamps do not replace the
   first passing candidate decoded for that camera.
5. Sort the retained candidates by timestamp and enumerate inclusive windows whose newest and
   oldest timestamps differ by at most `0.040 s`.
6. Select the temporal window with the most distinct cameras. Break ties by the newer window, then
   lower summed linear uncertainty, then configured camera order.
7. Reject valid candidates outside the chosen window as `TIMESTAMP_CLUSTER_NOT_SELECTED`.
8. Run spatial consensus on the selected coherent window.

One camera therefore contributes at most one vote, consensus never compares candidates outside the
approved 40 ms coherence window, and one corrupt future timestamp cannot suppress healthy cameras.
Timestamp validity is evaluated before any drivetrain pose-history lookup, initialization update,
or accepted-snapshot update.

Spatial consensus preserves the reference behavior:

- Candidates within `0.45 m` in field-relative XY are neighbors.
- The candidate with the largest neighborhood wins.
- Tied neighborhoods use the reference quality score derived from candidate uncertainty, neighbor
  distances, and neighbor uncertainty.
- The selected real camera pose retains its original timestamp and covariance.
- Poses are never averaged into a synthetic measurement.
- Every other coherent candidate is logged as `CONSENSUS_NOT_SELECTED`.
- At most one measurement enters the drive estimator per scheduler loop.

A single coherent candidate may win, matching the reference's ability to localize from one camera.

## Coplanarity Fix

PhotonVision and WPILib define a tag's local positive X axis as the normal pointing out of its
visible face. The reference rotates local positive Z, which remains vertical for this field layout
and incorrectly marks nearly every tag set as coplanar.

The corrected calculation rotates unit vector `(1, 0, 0)` by each tag's field rotation. Tags are
classified as coplanar when every known tag face normal is within `15 degrees` of the first known
normal. One tag is conservatively treated as coplanar. A missing first tag pose is also treated as
vulnerable; later unknown IDs are skipped, preserving the reference's conservative failure mode.

This classification controls both hybrid solver ordering and the multi-tag uncertainty penalty.

## Dynamic Measurement Confidence

Before covariance scaling, the reference normalizes confidence distance as:

```text
effectiveDistance = rawConfidenceDistance > 0.0 ? rawConfidenceDistance : 5.5
```

This means zero, negative, and NaN values use `5.5 m`; positive infinity has already failed the
distance gate. Translation standard deviation is then:

```text
max(1e-6,
    0.02
    * effectiveDistance^2 / max(tagCount, 1)
    * cameraFactor
    * aimingFactor
    * singleOrCoplanarFactor)
```

- All four camera factors remain `1.0`.
- `aimingFactor` is `0.6` while aiming and `1.0` otherwise.
- `singleOrCoplanarFactor` is `5.0` for one tag or a corrected-coplanar tag set and `1.0`
  otherwise.
- Heading standard deviation is `1e9`, so gyro heading remains authoritative during fusion.

The coplanar penalty remains controlled by the captured reference-compatible
`vision.applyCoplanarPenalty` setting, defaulting to true.

## Initialization

Initialization is tracked independently per camera and advances only from the consensus-selected
`PHOTONVISION_MULTITAG_COPROCESSOR` observation.

- Five stable observations from one camera complete initialization.
- Timestamps must strictly increase.
- Consecutive translation may change by at most `0.20 m`.
- Consecutive heading may change by at most `10 degrees`.
- An unstable observation restarts that camera's streak at one.
- Observations from another camera or another solver do not erase a camera's streak.

When initialization completes, every Photon IO instance exits its startup preference and begins
using the configured or hybrid strategy order. A successful disabled auto-reseed also marks
initialization complete.

## Accepted Snapshot and Reseeding

The latest accepted snapshot contains the selected pose, a defensive copy of tag IDs, and the
capture timestamp. It updates only for a strictly newer selected timestamp; equal or out-of-order
timestamps cannot replace it. Snapshots older than `0.5 s` are unavailable.

While disabled, the snapshot updates only from coprocessor multi-tag observations. Automatic
reseeding requires:

- A recent snapshot.
- At least two contributing tag IDs.
- An initial reset in the current disabled period or more than `0.25 m` translation drift.
- At least `0.5 s` since the prior automatic reset.

Leaving disabled clears the per-disabled-cycle initial-reset flag. Enabled operation never
automatically resets pose.

The snapshot intentionally preserves the reference edge at the enabled-to-disabled transition. A
recent enabled snapshot with at least two IDs remains eligible for the first disabled auto-reseed
until it expires or a newer disabled coprocessor multi-tag snapshot replaces it. The snapshot does
not retain solver type.

`forceReseedFromVision()` preserves the reference manual behavior: it resets from any recent
accepted snapshot and returns whether a snapshot was available. It does not require disabled mode
or multi-tag input, and it does not mark initialization complete. Operator Start+Back uses the
reference enabled-only command behavior; it does not call `ignoringDisable(true)`.

Both automatic and manual vision reseeds reset only the estimator. They never move the independent
simulation ground truth.

## Drive Integration and Timebase

`Drive` adds:

- `Optional<Pose2d> samplePoseAt(double timestampSeconds)`, delegating to the WPILib pose
  estimator's timestamped history.
- `getPitchDegrees()` and `getRollDegrees()`.
- A tilt-stability helper using the approved absolute threshold.
- An independent simulated-odometry pose getter.

Pose-reset APIs have distinct semantics:

- Existing `setPose(Pose2d)` resets only the estimator. Vision reseeds and driver Start+Back use
  this operation.
- `setPoseAndSimulationTruth(Pose2d)` calls `setPose()` and, only in SIM, resets the independent
  simulation odometry. AutoBuilder's pose-reset callback uses this operation.
- A test-only simulation pose hook may reset or override truth without routing through the vision
  estimator.

Photon capture timestamps and this target's `SwerveDrivePoseEstimator` both use FPGA seconds. The
CTRE `Utils.fpgaToCurrentTime()` conversions from the reference are deliberately omitted. Selected
timestamps pass unchanged to both `samplePoseAt()` and `addVisionMeasurement()`.

`GyroIOInputs` adds pitch and roll in degrees:

- `GyroIOPigeon2` refreshes Pigeon yaw, yaw rate, pitch, and roll before every read.
- Pitch and roll update at `50 Hz`; high-frequency odometry yaw remains unchanged.
- Bus-utilization optimization runs only after all update frequencies are configured.
- `GyroIONavX` populates the same pitch and roll fields for architecture parity.
- SIM no-op gyro inputs explicitly remain zero.
- REPLAY restores recorded pitch and roll through AdvantageKit.

## Independent Vision Simulation

Photon simulation uses one shared, instance-owned `VisionSystemSim` for the four cameras. It is
updated once per robot loop before camera inputs are read. Avoiding static global simulation state
prevents duplicate cameras and cross-test contamination.

Each simulated camera retains the reference properties:

- Resolution: `800 x 600`.
- Diagonal field of view: `72 degrees`.
- Calibration error: `(0.38, 0.1)`.
- Frame rate: `60 FPS`.
- Average latency: `10 ms`.
- Latency standard deviation: `5 ms`.

`Drive` maintains an independent odometry pose from gyro/module samples. Vision measurements and
vision resets cannot modify it. AutoBuilder is wired to `setPoseAndSimulationTruth()` so an
autonomous starting pose intentionally relocates both estimates. Normal `setPose()` calls,
including vision and driver heading recovery, do not rewrite truth.

`RobotContainer` exposes a simulation-pose supplier override for deterministic tests. Tests use an
explicit independent truth source and never generate detections from the estimator under test.

## RobotContainer Integration

`RobotContainer` retains one `Vision` subsystem and constructs:

- REAL: four `VisionIOPhotonVision` instances.
- SIM: one shared `VisionSimulation` and four `VisionIOPhotonVisionSim` instances.
- REPLAY: four `VisionIO.NoOp` instances.

A package-private pure `VisionConstructionPlan` maps `Constants.Mode` to the IO kind and the four
immutable camera configurations. `RobotContainer` realizes that plan at runtime. Tests inspect the
REAL plan without invoking its deferred Photon constructors, and instantiate only safe SIM/REPLAY
plans. This verifies mode mapping without constructing the existing Pigeon or TalonFX hardware.

The subsystem receives target-native drive callbacks/providers for weighted measurement, estimator
reset, current pose, timestamped pose, chassis speeds, pitch, and roll. No camera IO constructs CAN
hardware.

Operator Start+Back calls `forceReseedFromVision()` while enabled. The existing driver Start+Back
heading reset remains unchanged and separate. `setAiming(false)` is the startup default; no
placeholder Superstructure behavior is added.

## AdvantageKit Logging

Every scheduler loop:

1. Updates all camera IO.
2. Calls `Logger.processInputs("Vision/<fixed configured camera name>", inputs)` for each camera.
3. Clears transient output arrays and reasons.
4. Processes filtering, coherence, consensus, initialization, and reseeding.
5. Records per-camera and summary outputs.

Per-camera outputs include:

- Raw, accepted, rejected, superseded, skew-rejected, and selected robot poses.
- Contributing tag poses and IDs.
- Active strategy and observation type.
- Rejection reason.
- Confidence distance and ambiguity.
- Capture timestamp and timestamp skew.
- Pitch, roll, innovation, and connection alert.

Summary outputs include:

- All raw, accepted, rejected, and selected poses.
- Selected camera, candidate count, cluster size, uncertainty, innovation, and timestamp.
- Aiming state and drivetrain linear/angular speed.
- Per-camera initialization streaks and global completion.
- Latest accepted snapshot age and tag IDs.
- Disabled/manual reseed result, pose, delta, timestamp, and rejection reason.
- Vision-loop execution time.
- Accepted pose and tag-to-pose line arrays for AdvantageScope field visualization.

All per-frame arrays and strings are explicitly cleared when absent. Camera-specific keys prevent
later cameras from overwriting earlier telemetry.

## Error Handling

- A disconnected camera raises a warning alert without stopping other cameras or discarding
  already-buffered unread results.
- Individual missing tag transforms are ignored for distance aggregation; zero usable distance
  samples reject conservatively through positive infinity.
- Unknown strategy property tokens are ignored; an empty parsed order falls back to the complete
  explicitly documented fallback chain.
- Invalid distance-confidence properties fall back to all-tag average.
- Unknown tag IDs cannot crash coplanarity or field-overlay processing.
- Missing timestamped drive history makes trig/constrained solving unavailable for that frame and
  uses the current pose only for the innovation reference fallback.
- A stale or missing snapshot makes manual reseed return false and disabled auto-reseed do nothing.
- All snapshot and observation tag-ID arrays are defensively copied.
- REPLAY constructs no Photon camera or simulation objects.
- Fixed configured camera identities select replay log keys before inputs are restored.

## Testing

### Pure Filtering and Confidence Tests

Tests cover:

- Every hard gate, exact boundary, and a representative rejection reason.
- Infinity, zero, negative, NaN, partially missing, and entirely missing distance data, including
  the reference-compatible `5.5 m` covariance fallback.
- Default all-tag average, maximum-distance mode, and invalid-property fallback.
- Camera, aiming, single-tag, and corrected-coplanar uncertainty factors.
- `1e9` heading uncertainty and the `1e-6` translation floor.
- Field bounds from the active custom layout.
- Defensive copying of nested tag-ID arrays.

### Solver-Order Tests

Pure tests pin both approved startup orders and every hybrid branch:

- Single tag below and above the translation threshold.
- Multi-tag corrected-coplanar and noncoplanar geometry.
- Angular rates below `0.5`, between `0.5` and `1.0`, and above `1.0 rad/s`.
- Explicit valid, partial, invalid, and empty strategy-order properties.
- HYBRID and non-HYBRID strategy-mode paths.
- Property parsing through immutable configuration, without cross-test global state.

### Consensus Tests

Tests cover:

- No candidates and one candidate.
- Largest spatial cluster.
- Exact cluster-size and quality ties.
- Exactly `0.45 m` and just beyond the spatial boundary.
- Exactly `40 ms` and just beyond the timestamp boundary.
- Exactly `20 ms` future and `500 ms` old, plus just beyond both validity boundaries.
- Nonfinite and equal timestamps.
- Largest temporal cluster, newest-cluster tie break, uncertainty tie break, and camera-order tie
  break.
- Multiple unread frames from one camera.
- A stale camera backlog that previously outvoted current cameras.
- At most one candidate per camera and exactly one consumer call per loop.
- Preservation of the selected real pose, timestamp, and covariance.

### Coplanarity Tests

Tests rotate local positive X and verify:

- Tags on the same Hub face are classified coplanar.
- Tags on different Hub faces are not classified coplanar.
- The `15 degree` boundary.
- Single-tag and unknown-ID conservative behavior.
- Corrected classification affects both hybrid ordering and uncertainty.

### Initialization and Reseed Tests

Tests cover:

- Five-observation completion per camera.
- Increasing timestamps and translation/heading stability boundaries.
- Unstable restart at one.
- Isolation between cameras.
- Fallback observations not clearing a multi-tag streak.
- Snapshot freshness and defensive copies.
- Strictly newer snapshot replacement and out-of-order rejection.
- Disabled multi-tag restriction, initial reset, drift reset, interval limit, and enable-cycle reset.
- The preserved recent enabled multi-ID snapshot edge on transition to disabled.
- No enabled automatic reset.
- Manual success and failure without changing simulation truth.

### Photon Simulation and Strategy Comparison

The four-camera simulation uses independent truth and requires nonzero detections and accepted
observations. It exercises the eight reference stationary poses and representative translation and
rotation motion.

Both startup orders run the same deterministic scenarios and must satisfy:

- Maximum stationary estimator jump no greater than `0.15 m`.
- Maximum moving excess jump no greater than `0.25 m`.
- Zero moving cycles with excess jump above `0.40 m`.
- Observation coverage of at least `30 percent`.
- Nonzero accepted observations.

The winner is selected deterministically:

1. If only one order passes every safety gate, it wins.
2. If neither passes, verification fails and no order is accepted.
3. If both pass and mean accepted-pose errors differ by more than `0.02 m`, lower mean error wins.
4. Otherwise, if worst accepted-pose errors differ by more than `0.05 m`, lower worst error wins.
5. Otherwise, if coverage differs by more than `0.05` absolute, higher coverage wins.
6. Otherwise the constrained-second order wins.

Equality at a comparison tolerance advances to the next metric. Verification reports both raw
metric sets. After the first passing comparison, implementation sets and commits
`DEFAULT_STARTUP_STRATEGY_ORDER` to the computed winner. A normal regression test recomputes the
winner and fails if the compiled default no longer matches.

### Integration and Final Gate

Tests verify:

- REAL, SIM, and REPLAY construction plans without constructing CAN hardware in tests.
- Four exact names and transforms in every applicable mode.
- Shared simulation updates once per loop.
- Pigeon signal membership, `50 Hz` pitch/roll timing, and refresh-before-read behavior.
- Direct FPGA timestamp use without CTRE conversion.
- Operator Start+Back binding and unchanged driver Start+Back binding.
- Every AdvantageKit input field and required output key.

Pure vision and construction-plan tests remain in the normal Gradle `test` task. JNI/global-state
Photon integration runs in a dedicated `visionSimulationTest` Gradle `Test` task with
`forkEvery = 1`, wired into `check`. Tests do not repeatedly instantiate `RobotContainer` or
reconfigure global AutoBuilder in one JVM. Final verification runs focused vision tests,
`visionSimulationTest`, the complete repository suite, compilation, `spotlessCheck`, and
`git diff --check`.

## Acceptance Criteria

The port is complete when:

- The full production behavior and both approved fixes are present.
- The startup order is selected using the approved deterministic comparison rule.
- REAL, SIM, and REPLAY construct the correct IO without unintended hardware access.
- Four-camera simulation uses independent truth and produces nonzero coverage.
- Enabled vision never hard-resets pose.
- Consensus sends at most one timestamp-coherent real measurement per loop.
- All telemetry is live, camera-specific, and free of stale prior-frame values.
- Focused and full verification pass with clean formatting and diff hygiene.
