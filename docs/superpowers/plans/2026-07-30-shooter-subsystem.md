# Shooter Subsystem Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an AdvantageKit-native shooter subsystem with real, full-simulation, and replay IO
before future Superstructure integration.

**Architecture:** `Shooter` owns setpoints, readiness, and the STOP/PREPFUEL/SHOOT state machine.
One `ShooterIO` boundary carries commands and structured telemetry for the flywheel pair, kicker,
and hood. `ShooterIOTalonFX` owns four Phoenix controllers, while `ShooterIOSim` models the same four
devices and control modes.

**Tech Stack:** Java 17, WPILib 2026.2.1, AdvantageKit `@AutoLog`, CTRE Phoenix 6, JUnit 5,
GradleRIO.

## Global Constraints

- Use hood ID 13, kicker ID 14, flywheel lead ID 15, and opposed flywheel follower ID 16.
- Use only the hood TalonFX integrated encoder and reset it to 0.0 rotations at startup.
- Use `VelocityTorqueCurrentFOC` for flywheel RPM and `PositionVoltage` for hood position.
- Expose voltage, duty-cycle percentage, and torque-current FOC direct controls for flywheel,
  kicker, and hood.
- Configure direct `TorqueCurrentFOC` requests with 1 A deadband, 1.0 maximum absolute duty cycle,
  coast-during-neutral override, and 100 Hz updates.
- Configure `VelocityTorqueCurrentFOC` with coast-during-neutral override and 100 Hz updates;
  Phoenix does not expose deadband or maximum-duty fields on this request type.
- Configure voltage and duty-cycle requests with FOC enabled and 100 Hz updates.
- Configure hood position and follower requests at 100 Hz.
- Configure position, velocity, applied voltage, and stator current at 50 Hz.
- Configure temperature and all otherwise unspecified status signals at 4 Hz.
- Preserve flywheel limits of 100 A stator, 40 A supply, 20 A supply-lower after 0.25 seconds,
  ±12 V, +120/-40 A peak torque, and reference Slot 0 gains.
- Preserve kicker limits of 80 A stator, 40 A supply, 20 A supply-lower after 0.25 seconds,
  and ±12 V.
- Use 30 A stator, 15 A supply, ±10 V, and reference Slot 0 gains for the hood.
- Preserve the reference interpolation table and 1200 RPM PREPFUEL target.
- Flywheel ready means `targetRPM - 120 <= actualRPM <= targetRPM + 60`.
- Hood ready means position error is at most 0.125 rotations.
- STOP must command kicker 0 V immediately; do not add a clearing timer.
- Add full dynamic simulation for both flywheels, kicker, and hood.
- Add no controller bindings, default shooter command, Superstructure logic, autonomous named
  commands, CANcoder configuration, fuel sensing, or jam detection.
- Do not modify unrelated files or the existing hopper implementation.

---

## File Map

- `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`: IDs, typed limits, setpoints,
  interpolation, Phoenix configuration factories, and simulation constants.
- `src/main/java/frc/robot/subsystems/shooter/ShooterIO.java`: one logged input schema and command
  boundary for all four motors.
- `src/main/java/frc/robot/subsystems/shooter/Shooter.java`: public API, setpoint selection,
  readiness, state transitions, and AdvantageKit outputs.
- `src/main/java/frc/robot/subsystems/shooter/ShooterIOTalonFX.java`: four-controller Phoenix
  implementation, follower setup, signal refresh, and CAN optimization.
- `src/main/java/frc/robot/subsystems/shooter/ShooterIOSim.java`: four-motor physics and retained
  control modes.
- `src/main/java/frc/robot/RobotContainer.java`: retained shooter and mode-specific IO factory.
- `src/test/java/frc/robot/subsystems/shooter/ShooterTest.java`: state, readiness, interpolation,
  and direct-control tests.
- `src/test/java/frc/robot/subsystems/shooter/ShooterIOTalonFXConfigTest.java`: Phoenix configs,
  requests, follower, and frequency tests without hardware construction.
- `src/test/java/frc/robot/subsystems/shooter/ShooterIOSimTest.java`: dynamic telemetry and
  closed-loop simulation tests.
- `src/test/java/frc/robot/RobotContainerShooterTest.java`: simulation and replay factory tests.

---

### Task 1: Shooter Contract, Setpoints, and State Machine

**Files:**

- Create: `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`
- Create: `src/main/java/frc/robot/subsystems/shooter/ShooterIO.java`
- Create: `src/main/java/frc/robot/subsystems/shooter/Shooter.java`
- Test: `src/test/java/frc/robot/subsystems/shooter/ShooterTest.java`

**Interfaces:**

- Produces: `ShooterIO` methods
  `setFlywheelVelocity(double)`, `setFlywheelVoltage(Voltage)`,
  `setFlywheelDutyCycle(double)`, `setFlywheelTorqueCurrent(Current)`,
  `setKickerVoltage(Voltage)`, `setKickerDutyCycle(double)`,
  `setKickerTorqueCurrent(Current)`, `setHoodPosition(double)`,
  `setHoodVoltage(Voltage)`, `setHoodDutyCycle(double)`,
  `setHoodTorqueCurrent(Current)`, and `resetHoodPosition(double)`.
- Produces: `Shooter(ShooterIO)`, direct-control methods bearing the same mechanism names,
  `setDesiredState(ShooterState)`, `setSetpoint(double, double)`,
  `setSetpointForDistance(double)`, state/target accessors, and readiness accessors.
- Produces: `ShooterState.STOP`, `PREPFUEL`, `SHOOT`, and internal transition state `TRANSITION`.

- [ ] **Step 1: Load the test-first rules**

Read completely before editing:

```text
C:\Users\dougd\.codex\plugins\cache\openai-curated-remote\superpowers\6.2.0\skills\test-driven-development\SKILL.md
C:\Users\dougd\.codex\plugins\cache\openai-curated-remote\superpowers\6.2.0\skills\test-driven-development\writing-good-tests.md
```

- [ ] **Step 2: Write failing state and readiness tests**

Create `ShooterTest.java` with a recording IO that publishes measured flywheel and hood values:

```java
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import org.junit.jupiter.api.Test;

class ShooterTest {
  private static class RecordingIO implements ShooterIO {
    double measuredFlywheelRpm;
    double measuredHoodRotations;
    double flywheelVelocityRpm;
    double flywheelVolts;
    double flywheelDutyCycle;
    double flywheelTorqueAmps;
    double kickerVolts;
    double kickerDutyCycle;
    double kickerTorqueAmps;
    double hoodPositionRotations;
    double hoodVolts;
    double hoodDutyCycle;
    double hoodTorqueAmps;
    double resetHoodRotations = Double.NaN;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      inputs.flywheelLeadVelocityRpm = measuredFlywheelRpm;
      inputs.hoodPositionRotations = measuredHoodRotations;
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
      flywheelVelocityRpm = rpm;
    }

    @Override
    public void setFlywheelVoltage(Voltage voltage) {
      flywheelVolts = voltage.in(Volts);
    }

    @Override
    public void setFlywheelDutyCycle(double output) {
      flywheelDutyCycle = output;
    }

    @Override
    public void setFlywheelTorqueCurrent(Current current) {
      flywheelTorqueAmps = current.in(Amps);
    }

    @Override
    public void setKickerVoltage(Voltage voltage) {
      kickerVolts = voltage.in(Volts);
    }

    @Override
    public void setKickerDutyCycle(double output) {
      kickerDutyCycle = output;
    }

    @Override
    public void setKickerTorqueCurrent(Current current) {
      kickerTorqueAmps = current.in(Amps);
    }

    @Override
    public void setHoodPosition(double rotations) {
      hoodPositionRotations = rotations;
    }

    @Override
    public void setHoodVoltage(Voltage voltage) {
      hoodVolts = voltage.in(Volts);
    }

    @Override
    public void setHoodDutyCycle(double output) {
      hoodDutyCycle = output;
    }

    @Override
    public void setHoodTorqueCurrent(Current current) {
      hoodTorqueAmps = current.in(Amps);
    }

    @Override
    public void resetHoodPosition(double rotations) {
      resetHoodRotations = rotations;
    }
  }

  @Test
  void constructorZerosHoodAndFirstPeriodicStopsSafely() {
    RecordingIO io = new RecordingIO();
    Shooter shooter = new Shooter(io);

    shooter.periodic();

    assertEquals(0.0, io.resetHoodRotations, 1e-9);
    assertEquals(0.0, io.flywheelVolts, 1e-9);
    assertEquals(0.0, io.kickerVolts, 1e-9);
    assertEquals(0.0, io.hoodPositionRotations, 1e-9);
    assertEquals(ShooterState.STOP, shooter.getCurrentState());
  }

  @Test
  void prepFuelUsesLockedOutputsAndReadinessWindow() {
    RecordingIO io = new RecordingIO();
    Shooter shooter = new Shooter(io);
    shooter.periodic();
    shooter.setDesiredState(ShooterState.PREPFUEL);
    io.measuredFlywheelRpm = 1080.0;

    shooter.periodic();

    assertEquals(1200.0, io.flywheelVelocityRpm, 1e-9);
    assertEquals(6.0, io.kickerVolts, 1e-9);
    assertEquals(0.0, io.hoodPositionRotations, 1e-9);
    assertEquals(ShooterState.PREPFUEL, shooter.getCurrentState());
  }

  @Test
  void shootWaitsForFlywheelAndHoodThenFeeds() {
    RecordingIO io = new RecordingIO();
    Shooter shooter = new Shooter(io);
    shooter.periodic();
    shooter.setSetpoint(2500.0, 0.75);
    shooter.setDesiredState(ShooterState.SHOOT);
    io.measuredFlywheelRpm = 2380.0;
    io.measuredHoodRotations = 0.625;

    shooter.periodic();
    assertEquals(ShooterState.SHOOT, shooter.getCurrentState());
    assertEquals(6.0, io.kickerVolts, 1e-9);

    shooter.periodic();
    assertEquals(12.0, io.kickerVolts, 1e-9);
  }

  @Test
  void readinessUsesAsymmetricInclusiveFlywheelBounds() {
    RecordingIO io = new RecordingIO();
    Shooter shooter = new Shooter(io);
    shooter.setSetpoint(2500.0, 0.0);

    io.measuredFlywheelRpm = 2380.0;
    shooter.periodic();
    assertTrue(shooter.isFlywheelReady());
    io.measuredFlywheelRpm = 2560.0;
    shooter.periodic();
    assertTrue(shooter.isFlywheelReady());
    io.measuredFlywheelRpm = 2379.99;
    shooter.periodic();
    assertFalse(shooter.isFlywheelReady());
    io.measuredFlywheelRpm = 2560.01;
    shooter.periodic();
    assertFalse(shooter.isFlywheelReady());
  }

  @Test
  void hoodReadinessUsesInclusivePointOneTwoFiveRotationTolerance() {
    RecordingIO io = new RecordingIO();
    Shooter shooter = new Shooter(io);
    shooter.setSetpoint(2500.0, 0.75);

    io.measuredHoodRotations = 0.625;
    shooter.periodic();
    assertTrue(shooter.isHoodReady());
    io.measuredHoodRotations = 0.875;
    shooter.periodic();
    assertTrue(shooter.isHoodReady());
    io.measuredHoodRotations = 0.6249;
    shooter.periodic();
    assertFalse(shooter.isHoodReady());
  }

  @Test
  void stopHaltsKickerImmediatelyWithoutClearingDelay() {
    RecordingIO io = new RecordingIO();
    Shooter shooter = new Shooter(io);
    shooter.periodic();
    shooter.setDesiredState(ShooterState.SHOOT);
    io.measuredFlywheelRpm = 2500.0;
    shooter.periodic();
    shooter.periodic();
    assertEquals(12.0, io.kickerVolts, 1e-9);

    shooter.setDesiredState(ShooterState.STOP);
    shooter.periodic();

    assertEquals(0.0, io.kickerVolts, 1e-9);
  }

  @Test
  void interpolationPreservesReferenceEndpoints() {
    RecordingIO io = new RecordingIO();
    Shooter shooter = new Shooter(io);

    shooter.setSetpointForDistance(edu.wpi.first.math.util.Units.feetToMeters(5.0));
    assertEquals(2400.0, shooter.getTargetRpm(), 1e-9);
    assertEquals(0.0, shooter.getTargetHoodRotations(), 1e-9);

    shooter.setSetpointForDistance(edu.wpi.first.math.util.Units.feetToMeters(12.0));
    assertEquals(3000.0, shooter.getTargetRpm(), 1e-9);
    assertEquals(1.25, shooter.getTargetHoodRotations(), 1e-9);
  }

  @Test
  void directControlsClampAtSubsystemBoundary() {
    RecordingIO io = new RecordingIO();
    Shooter shooter = new Shooter(io);

    shooter.runFlywheelVoltage(Volts.of(20.0));
    shooter.runFlywheelPercentage(-2.0);
    shooter.runFlywheelTorqueCurrentFOC(Amps.of(-80.0));
    shooter.runKickerTorqueCurrentFOC(Amps.of(100.0));
    shooter.runHoodVoltage(Volts.of(-12.0));
    shooter.runHoodTorqueCurrentFOC(Amps.of(50.0));

    assertEquals(12.0, io.flywheelVolts, 1e-9);
    assertEquals(-1.0, io.flywheelDutyCycle, 1e-9);
    assertEquals(-40.0, io.flywheelTorqueAmps, 1e-9);
    assertEquals(80.0, io.kickerTorqueAmps, 1e-9);
    assertEquals(-10.0, io.hoodVolts, 1e-9);
    assertEquals(30.0, io.hoodTorqueAmps, 1e-9);
  }

  @Test
  void nullDesiredStateIsRejected() {
    Shooter shooter = new Shooter(new RecordingIO());
    assertThrows(NullPointerException.class, () -> shooter.setDesiredState(null));
    assertThrows(
        IllegalArgumentException.class,
        () -> shooter.setDesiredState(ShooterState.TRANSITION));
  }
}
```

- [ ] **Step 3: Run the focused test and verify RED**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.shooter.ShooterTest
```

Expected: Java compilation fails because the shooter package does not exist.

- [ ] **Step 4: Add constants and the IO contract**

Create `ShooterConstants.java` with typed fields for every Global Constraint. Define:

```java
static final int HOOD_MOTOR_ID = 13;
static final int KICKER_MOTOR_ID = 14;
static final int FLYWHEEL_LEAD_MOTOR_ID = 15;
static final int FLYWHEEL_FOLLOWER_MOTOR_ID = 16;

static final Current FLYWHEEL_STATOR_LIMIT = Amps.of(100.0);
static final Current FLYWHEEL_FORWARD_TORQUE_LIMIT = Amps.of(120.0);
static final Current FLYWHEEL_REVERSE_TORQUE_LIMIT = Amps.of(-40.0);
static final Current KICKER_STATOR_LIMIT = Amps.of(80.0);
static final Current HOOD_STATOR_LIMIT = Amps.of(30.0);
static final Voltage FLYWHEEL_MAX_VOLTAGE = Volts.of(12.0);
static final Voltage KICKER_MAX_VOLTAGE = Volts.of(12.0);
static final Voltage HOOD_MAX_VOLTAGE = Volts.of(10.0);
static final double DEFAULT_FLYWHEEL_RPM = 2500.0;
static final double PREP_FLYWHEEL_RPM = 1200.0;
static final double READY_BELOW_RPM = 120.0;
static final double READY_ABOVE_RPM = 60.0;
static final double STOPPED_TOLERANCE_RPM = 0.5;
static final double DEFAULT_HOOD_ROTATIONS = 0.0;
static final double HOOD_READY_TOLERANCE_ROTATIONS = 0.125;
static final Voltage KICKER_PREP_VOLTAGE = Volts.of(6.0);
static final Voltage KICKER_SHOOT_VOLTAGE = Volts.of(12.0);
```

Populate `InterpolatingDoubleTreeMap` instances with all eight approved rows. Add:

```java
record ShooterSetpoint(double flywheelRpm, double hoodRotations) {}

static ShooterSetpoint getSetpointForDistance(double distanceMeters) {
  return new ShooterSetpoint(
      FLYWHEEL_RPM_MAP.get(distanceMeters), HOOD_ROTATIONS_MAP.get(distanceMeters));
}
```

Create `ShooterIO.java` with a nested `NoOp` implementation, an `@AutoLog` input class, and the
exact methods from the Interfaces block. The input class has these six fields for each prefix
`flywheelLead`, `flywheelFollower`, `kicker`, and `hood`:

```java
public boolean flywheelLeadConnected;
public double flywheelLeadPositionRotations;
public double flywheelLeadVelocityRpm;
public double flywheelLeadAppliedVolts;
public double flywheelLeadCurrentAmps;
public double flywheelLeadTempCelsius;
```

Repeat the six concrete fields using the other three prefixes; do not use arrays because named
AdvantageKit fields are required.

- [ ] **Step 5: Implement the state machine**

Create `Shooter.java` with initial state:

```java
private ShooterState desiredState = ShooterState.STOP;
private ShooterState currentState = ShooterState.TRANSITION;
private double targetRpm = DEFAULT_FLYWHEEL_RPM;
private double targetHoodRotations = DEFAULT_HOOD_ROTATIONS;
```

The constructor stores a non-null IO and calls:

```java
io.resetHoodPosition(DEFAULT_HOOD_ROTATIONS);
```

Implement the asymmetric helper exactly:

```java
private boolean isFlywheelReadyFor(double requestedRpm) {
  double actualRpm = inputs.flywheelLeadVelocityRpm;
  return actualRpm >= requestedRpm - READY_BELOW_RPM
      && actualRpm <= requestedRpm + READY_ABOVE_RPM;
}
```

Use this transition switch:

```java
private void handleState() {
  switch (currentState) {
    case STOP -> {}
    case PREPFUEL -> commandPrepFuel();
    case SHOOT -> commandShoot();
    case TRANSITION -> {
      switch (desiredState) {
        case STOP -> transitionToStop();
        case PREPFUEL -> transitionToPrepFuel();
        case SHOOT -> transitionToShoot();
        case TRANSITION -> throw new IllegalStateException("TRANSITION cannot be desired");
      }
    }
  }
}
```

`transitionToStop()` commands flywheel 0 V, kicker 0 V, and hood 0 rotations on every transition
loop, then selects STOP when `abs(flywheelLeadVelocityRpm) < 0.5`.
Steady STOP sends no additional request, so a deliberate direct-control call can remain active
until another state is requested. Startup is still safe because the initial current state is TRANSITION.

`transitionToPrepFuel()` and steady PREPFUEL command 1200 RPM, kicker 6 V, and hood 0 rotations.
Transition to PREPFUEL when `isFlywheelReadyFor(1200.0)` is true.

`transitionToShoot()` commands selected RPM, selected hood rotations, and kicker 6 V. It selects
SHOOT only when `isFlywheelReadyFor(targetRpm)` and
`MathUtil.isNear(targetHoodRotations, inputs.hoodPositionRotations, 0.125)` are both true.
Steady SHOOT repeats flywheel and hood closed-loop requests and commands kicker 12 V.

`setDesiredState()` accepts only STOP, PREPFUEL, and SHOOT. Reject `null` with
`Objects.requireNonNull`; reject TRANSITION with `IllegalArgumentException`. When the accepted
request differs from current state, set `currentState = TRANSITION`.

In `periodic()`:

```java
io.updateInputs(inputs);
Logger.processInputs("Shooter", inputs);
handleState();
Logger.recordOutput("Shooter/CurrentState", currentState.name());
Logger.recordOutput("Shooter/DesiredState", desiredState.name());
Logger.recordOutput("Shooter/TargetRPM", targetRpm);
Logger.recordOutput("Shooter/TargetHoodRotations", targetHoodRotations);
Logger.recordOutput("Shooter/FlywheelReady", isFlywheelReady());
Logger.recordOutput("Shooter/HoodReady", isHoodReady());
```

Public direct methods clamp before forwarding. Flywheel torque clamps to -40 A through +100 A
(the effective intersection of peak-torque and stator limits), kicker torque to ±80 A, and hood
torque to ±30 A.

- [ ] **Step 6: Run the focused test and verify GREEN**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.shooter.ShooterTest
```

Expected: all shooter contract and state tests pass.

- [ ] **Step 7: Commit the subsystem contract**

```powershell
git add src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java
git add src/main/java/frc/robot/subsystems/shooter/ShooterIO.java
git add src/main/java/frc/robot/subsystems/shooter/Shooter.java
git add src/test/java/frc/robot/subsystems/shooter/ShooterTest.java
git commit -m "feat(shooter): add state and IO contract"
```

---

### Task 2: TalonFX Hardware IO and CAN Scheduling

**Files:**

- Modify: `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`
- Create: `src/main/java/frc/robot/subsystems/shooter/ShooterIOTalonFX.java`
- Test: `src/test/java/frc/robot/subsystems/shooter/ShooterIOTalonFXConfigTest.java`

**Interfaces:**

- Consumes: all `ShooterIO` methods and `PhoenixUtil.tryUntilOk`.
- Produces: `ShooterIOTalonFX`, package-private Phoenix config/request factories,
  `StatusFrequencyConfig(double mechanismHz, double temperatureHz, double unspecifiedHz)`, and
  four-motor temperature refresh groups.

- [ ] **Step 1: Write failing Phoenix configuration tests**

Create `ShooterIOTalonFXConfigTest.java`:

```java
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Celsius;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Temperature;
import org.junit.jupiter.api.Test;

class ShooterIOTalonFXConfigTest {
  @Test
  void hardwareIdsMatchReference() {
    assertEquals(13, ShooterConstants.HOOD_MOTOR_ID);
    assertEquals(14, ShooterConstants.KICKER_MOTOR_ID);
    assertEquals(15, ShooterConstants.FLYWHEEL_LEAD_MOTOR_ID);
    assertEquals(16, ShooterConstants.FLYWHEEL_FOLLOWER_MOTOR_ID);
  }

  @Test
  void flywheelVelocityRequestUsesSupportedLockedSettings() {
    VelocityTorqueCurrentFOC request = ShooterIOTalonFX.createFlywheelVelocityRequest();
    assertTrue(request.OverrideCoastDurNeutral);
    assertEquals(100.0, request.UpdateFreqHz, 1e-9);
  }

  @Test
  void directRequestsMatchHopperSettings() {
    VoltageOut voltage = ShooterIOTalonFX.createVoltageRequest();
    DutyCycleOut duty = ShooterIOTalonFX.createDutyCycleRequest();
    TorqueCurrentFOC torque = ShooterIOTalonFX.createTorqueCurrentRequest();
    assertTrue(voltage.EnableFOC);
    assertTrue(duty.EnableFOC);
    assertEquals(100.0, voltage.UpdateFreqHz, 1e-9);
    assertEquals(100.0, duty.UpdateFreqHz, 1e-9);
    assertEquals(1.0, torque.MaxAbsDutyCycle, 1e-9);
    assertEquals(1.0, torque.Deadband, 1e-9);
    assertTrue(torque.OverrideCoastDurNeutral);
    assertEquals(100.0, torque.UpdateFreqHz, 1e-9);
  }

  @Test
  void hoodPositionAndFollowerRequestsUseLockedSettings() {
    PositionVoltage hood = ShooterIOTalonFX.createHoodPositionRequest();
    Follower follower = ShooterIOTalonFX.createFlywheelFollowerRequest();
    assertTrue(hood.EnableFOC);
    assertEquals(100.0, hood.UpdateFreqHz, 1e-9);
    assertEquals(15, follower.LeaderID);
    assertEquals(MotorAlignmentValue.Opposed, follower.MotorAlignment);
    assertEquals(100.0, follower.UpdateFreqHz, 1e-9);
  }

  @Test
  void configsPreserveCurrentVoltageNeutralAndGains() {
    var lead = ShooterConstants.createFlywheelLeadConfig();
    var follower = ShooterConstants.createFlywheelFollowerConfig();
    var kicker = ShooterConstants.createKickerConfig();
    var hood = ShooterConstants.createHoodConfig();

    assertEquals(100.0, lead.CurrentLimits.StatorCurrentLimit, 1e-9);
    assertEquals(40.0, lead.CurrentLimits.SupplyCurrentLimit, 1e-9);
    assertEquals(20.0, lead.CurrentLimits.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.25, lead.CurrentLimits.SupplyCurrentLowerTime, 1e-9);
    assertEquals(120.0, lead.TorqueCurrent.PeakForwardTorqueCurrent, 1e-9);
    assertEquals(-40.0, lead.TorqueCurrent.PeakReverseTorqueCurrent, 1e-9);
    assertEquals(8.0, lead.Slot0.kP, 1e-9);
    assertEquals(4.325, lead.Slot0.kS, 1e-9);
    assertEquals(0.013, lead.Slot0.kV, 1e-9);
    assertEquals(InvertedValue.Clockwise_Positive, lead.MotorOutput.Inverted);
    assertEquals(NeutralModeValue.Coast, follower.MotorOutput.NeutralMode);

    assertEquals(80.0, kicker.CurrentLimits.StatorCurrentLimit, 1e-9);
    assertEquals(40.0, kicker.CurrentLimits.SupplyCurrentLimit, 1e-9);
    assertEquals(30.0, hood.CurrentLimits.StatorCurrentLimit, 1e-9);
    assertEquals(15.0, hood.CurrentLimits.SupplyCurrentLimit, 1e-9);
    assertEquals(10.0, hood.Voltage.PeakForwardVoltage, 1e-9);
    assertEquals(8.0, hood.Slot0.kP, 1e-9);
    assertEquals(0.1, hood.Slot0.kD, 1e-9);
    assertEquals(0.4, hood.Slot0.kG, 1e-9);
    assertEquals(NeutralModeValue.Brake, hood.MotorOutput.NeutralMode);
  }

  @Test
  void statusFrequenciesUseApprovedCanSchedule() {
    var frequencies = ShooterIOTalonFX.createStatusFrequencyConfig();
    assertEquals(50.0, frequencies.mechanismHz(), 1e-9);
    assertEquals(4.0, frequencies.temperatureHz(), 1e-9);
    assertEquals(4.0, frequencies.unspecifiedHz(), 1e-9);
  }

  @Test
  void temperatureRefreshGroupContainsAllFourSignals() {
    StatusSignal<Temperature> lead =
        new StatusSignal<>(StatusCode.StatusCodeNotInitialized, Temperature.class, Celsius::of);
    StatusSignal<Temperature> follower =
        new StatusSignal<>(StatusCode.StatusCodeNotInitialized, Temperature.class, Celsius::of);
    StatusSignal<Temperature> kicker =
        new StatusSignal<>(StatusCode.StatusCodeNotInitialized, Temperature.class, Celsius::of);
    StatusSignal<Temperature> hood =
        new StatusSignal<>(StatusCode.StatusCodeNotInitialized, Temperature.class, Celsius::of);

    assertArrayEquals(
        new BaseStatusSignal[] {lead, follower, kicker, hood},
        ShooterIOTalonFX.createTemperatureRefreshSignals(lead, follower, kicker, hood));
  }
}
```

- [ ] **Step 2: Run the hardware configuration test and verify RED**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.shooter.ShooterIOTalonFXConfigTest
```

Expected: Java compilation fails because `ShooterIOTalonFX` and config factories do not exist.

- [ ] **Step 3: Add exact TalonFX configuration factories**

Add four package-private factories to `ShooterConstants`. The flywheel lead config applies current,
voltage, motor output, Slot 0, and torque-current groups:

```java
return new TalonFXConfiguration()
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(FLYWHEEL_STATOR_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(40.0))
            .withSupplyCurrentLowerLimit(Amps.of(20.0))
            .withSupplyCurrentLowerTime(Seconds.of(0.25)))
    .withVoltage(
        new VoltageConfigs()
            .withPeakForwardVoltage(FLYWHEEL_MAX_VOLTAGE)
            .withPeakReverseVoltage(FLYWHEEL_MAX_VOLTAGE.unaryMinus()))
    .withMotorOutput(
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive))
    .withSlot0(
        new Slot0Configs().withKP(8.0).withKS(4.325).withKV(0.013))
    .withTorqueCurrent(
        new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(FLYWHEEL_FORWARD_TORQUE_LIMIT)
            .withPeakReverseTorqueCurrent(FLYWHEEL_REVERSE_TORQUE_LIMIT));
```

The follower config repeats flywheel current, voltage, coast, and torque-current groups, without
Slot 0 gains. The kicker config uses 80/40/20 A, 0.25 seconds, ±12 V, clockwise-positive, and coast.
The hood config uses 30/15 A, ±10 V, clockwise-positive, brake, and:

```java
new Slot0Configs()
    .withKP(8.0)
    .withKD(0.1)
    .withKG(0.4)
    .withGravityType(GravityTypeValue.Elevator_Static)
    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
```

- [ ] **Step 4: Implement request factories and hardware ownership**

Create `ShooterIOTalonFX.java`. Request factories are:

```java
static VelocityTorqueCurrentFOC createFlywheelVelocityRequest() {
  return new VelocityTorqueCurrentFOC(0.0)
      .withOverrideCoastDurNeutral(true)
      .withUpdateFreqHz(100.0);
}

static PositionVoltage createHoodPositionRequest() {
  return new PositionVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(100.0);
}

static VoltageOut createVoltageRequest() {
  return new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(100.0);
}

static DutyCycleOut createDutyCycleRequest() {
  return new DutyCycleOut(0.0).withEnableFOC(true).withUpdateFreqHz(100.0);
}

static TorqueCurrentFOC createTorqueCurrentRequest() {
  return new TorqueCurrentFOC(0.0)
      .withMaxAbsDutyCycle(1.0)
      .withDeadband(Amps.of(1.0))
      .withOverrideCoastDurNeutral(true)
      .withUpdateFreqHz(100.0);
}

static Follower createFlywheelFollowerRequest() {
  return new Follower(FLYWHEEL_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)
      .withUpdateFreqHz(100.0);
}
```

Own four TalonFX objects, one set of request objects per independently commanded mechanism, five
signals per motor (position, velocity, motor voltage, stator current, device temperature), and four
falling-edge 0.5-second connection debouncers.

Provide:

```java
static BaseStatusSignal[] createTemperatureRefreshSignals(
    StatusSignal<Temperature> lead,
    StatusSignal<Temperature> follower,
    StatusSignal<Temperature> kicker,
    StatusSignal<Temperature> hood) {
  return new BaseStatusSignal[] {lead, follower, kicker, hood};
}
```

Constructor order:

1. Apply the four configs with `tryUntilOk(5, () -> configurator.apply(config, 0.25))`.
2. Clear all four sticky-fault sets through the same retry helper.
3. Set all non-temperature signals to 50 Hz.
4. Set all four temperature signals to 4 Hz.
5. Call `ParentDevice.optimizeBusUtilizationForAll(4.0, lead, follower, kicker, hood)`.
6. Apply the opposed follower request with the retry helper.

- [ ] **Step 5: Implement refresh and output methods**

In `updateInputs`, call one `BaseStatusSignal.refreshAll(...)` for each motor's four 50 Hz signals,
plus one explicit refresh for the four temperature signals. Derive each connected flag from the
corresponding four-signal status and debouncer. Convert rotations/second to RPM by multiplying by
60.0, then assign every named input field.

Output methods use the exact request type and clamp:

```java
flywheelLeadMotor.setControl(
    flywheelVelocityRequest.withVelocity(rpm / 60.0));

flywheelLeadMotor.setControl(
    flywheelVoltageRequest.withOutput(MathUtil.clamp(voltage.in(Volts), -12.0, 12.0)));

kickerMotor.setControl(
    kickerTorqueRequest.withOutput(MathUtil.clamp(current.in(Amps), -80.0, 80.0)));

hoodMotor.setControl(
    hoodPositionRequest.withPosition(rotations));
```

Implement every other `ShooterIO` output with the corresponding motor, request object, and Global
Constraint clamp. `resetHoodPosition` performs the one startup reset requested by the `Shooter`
constructor:

```java
tryUntilOk(5, () -> hoodMotor.setPosition(rotations, 0.25));
```

- [ ] **Step 6: Run hardware and shooter regression tests**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.shooter.ShooterIOTalonFXConfigTest
.\gradlew.bat test --tests "frc.robot.subsystems.shooter.*"
```

Expected: request/configuration tests and Task 1 tests all pass without constructing CAN hardware
inside tests.

- [ ] **Step 7: Commit hardware IO**

```powershell
git add src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java
git add src/main/java/frc/robot/subsystems/shooter/ShooterIOTalonFX.java
git add src/test/java/frc/robot/subsystems/shooter/ShooterIOTalonFXConfigTest.java
git commit -m "feat(shooter): add TalonFX hardware IO"
```

---

### Task 3: Full Shooter Simulation

**Files:**

- Modify: `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`
- Create: `src/main/java/frc/robot/subsystems/shooter/ShooterIOSim.java`
- Test: `src/test/java/frc/robot/subsystems/shooter/ShooterIOSimTest.java`

**Interfaces:**

- Consumes: all `ShooterIO` methods and typed mechanism limits.
- Produces: four independent `DCMotorSim` models supporting flywheel velocity, hood position, and
  all three direct control modes.

- [ ] **Step 1: Write failing full-simulation tests**

Create `ShooterIOSimTest.java`:

```java
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ShooterIOSimTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void flywheelVelocityControlMovesOpposedMotorsTowardTarget() {
    ShooterIOSim io = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    io.setFlywheelVelocity(2500.0);

    for (int i = 0; i < 250; i++) {
      io.updateInputs(inputs);
    }

    assertTrue(inputs.flywheelLeadVelocityRpm > 2200.0);
    assertTrue(inputs.flywheelLeadVelocityRpm < 2700.0);
    assertTrue(inputs.flywheelFollowerVelocityRpm < -2200.0);
  }

  @Test
  void hoodPositionControlApproachesRequestedRotations() {
    ShooterIOSim io = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    io.setHoodPosition(0.75);

    for (int i = 0; i < 250; i++) {
      io.updateInputs(inputs);
    }

    assertEquals(0.75, inputs.hoodPositionRotations, 0.125);
  }

  @Test
  void kickerAndDirectModesProduceDynamicTelemetry() {
    ShooterIOSim io = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    io.setKickerDutyCycle(2.0);
    io.setHoodTorqueCurrent(Amps.of(50.0));
    io.setFlywheelVoltage(Volts.of(-20.0));

    for (int i = 0; i < 20; i++) {
      io.updateInputs(inputs);
    }

    assertEquals(12.0, inputs.kickerAppliedVolts, 1e-9);
    assertTrue(inputs.kickerVelocityRpm > 0.0);
    assertTrue(inputs.hoodCurrentAmps <= 30.0 + 1e-6);
    assertEquals(-12.0, inputs.flywheelLeadAppliedVolts, 1e-9);
    assertTrue(inputs.flywheelLeadVelocityRpm < 0.0);
  }

  @Test
  void everyConnectionAndTemperatureFieldIsExplicit() {
    ShooterIOSim io = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    inputs.flywheelLeadTempCelsius = 41.0;
    inputs.flywheelFollowerTempCelsius = 42.0;
    inputs.kickerTempCelsius = 43.0;
    inputs.hoodTempCelsius = 44.0;

    io.updateInputs(inputs);

    assertTrue(inputs.flywheelLeadConnected);
    assertTrue(inputs.flywheelFollowerConnected);
    assertTrue(inputs.kickerConnected);
    assertTrue(inputs.hoodConnected);
    assertEquals(0.0, inputs.flywheelLeadTempCelsius, 1e-9);
    assertEquals(0.0, inputs.flywheelFollowerTempCelsius, 1e-9);
    assertEquals(0.0, inputs.kickerTempCelsius, 1e-9);
    assertEquals(0.0, inputs.hoodTempCelsius, 1e-9);
  }
}
```

- [ ] **Step 2: Run simulation tests and verify RED**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.shooter.ShooterIOSimTest
```

Expected: Java compilation fails because `ShooterIOSim` does not exist.

- [ ] **Step 3: Add simulation constants**

Add:

```java
static final DCMotor FLYWHEEL_SIM_MOTOR = DCMotor.getKrakenX60Foc(1);
static final DCMotor KICKER_SIM_MOTOR = DCMotor.getKrakenX60Foc(1);
static final DCMotor HOOD_SIM_MOTOR = DCMotor.getKrakenX44Foc(1);
static final double SIM_GEARING = 1.0;
static final double FLYWHEEL_SIM_MOI = 0.004;
static final double KICKER_SIM_MOI = 0.003;
static final double HOOD_SIM_MOI = 0.01;
static final double LOOP_PERIOD_SECONDS = 0.02;
```

These inertias are simulation-only approximations because the reference contains no measured
mechanism inertia or gearing.

- [ ] **Step 4: Implement retained control modes and physics**

Create `ShooterIOSim.java` with four `DCMotorSim` instances. Use:

```java
private enum FlywheelControlMode { VOLTAGE, DUTY_CYCLE, TORQUE_CURRENT, VELOCITY }
private enum KickerControlMode { VOLTAGE, DUTY_CYCLE, TORQUE_CURRENT }
private enum HoodControlMode { VOLTAGE, DUTY_CYCLE, TORQUE_CURRENT, POSITION }
```

Each setter selects its mechanism's mode and stores a clamped request. Convert torque current to
voltage with the motor model:

```java
private static double torqueCurrentToVoltage(
    DCMotor motor, DCMotorSim sim, double currentAmps, double maxVoltage) {
  return MathUtil.clamp(
      motor.getVoltage(
          motor.getTorque(currentAmps),
          sim.getAngularVelocityRadPerSec() * SIM_GEARING),
      -maxVoltage,
      maxVoltage);
}
```

Flywheel velocity voltage uses motor free speed plus proportional correction:

```java
double feedforwardVolts =
    12.0
        * requestedFlywheelRpm
        / Units.radiansPerSecondToRotationsPerMinute(
            FLYWHEEL_SIM_MOTOR.freeSpeedRadPerSec);
double feedbackVolts =
    0.0015 * (requestedFlywheelRpm - flywheelLeadSim.getAngularVelocityRPM());
double leadVolts = MathUtil.clamp(feedforwardVolts + feedbackVolts, -12.0, 12.0);
double followerVolts = -leadVolts;
```

Hood position voltage uses position error and velocity damping:

```java
double positionError = requestedHoodRotations - hoodSim.getAngularPositionRotations();
double hoodVolts =
    MathUtil.clamp(
        8.0 * positionError - 0.02 * hoodSim.getAngularVelocityRPM(),
        -10.0,
        10.0);
```

Apply input voltages, update all four sims by 0.02 seconds, and assign all 24 telemetry fields. Use
absolute current draw, publish the follower's negative velocity/position, set every connection true,
and set every temperature to 0.0. `resetHoodPosition(rotations)` calls:

```java
hoodSim.setState(edu.wpi.first.math.util.Units.rotationsToRadians(rotations), 0.0);
```

- [ ] **Step 5: Run simulation and full shooter tests**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.shooter.ShooterIOSimTest
.\gradlew.bat test --tests "frc.robot.subsystems.shooter.*"
```

Expected: all simulation, state, and hardware-configuration tests pass. Tune only the two
simulation feedback coefficients when the physical assertions fail; do not change production
setpoints, tolerances, limits, or request configuration.

- [ ] **Step 6: Commit simulation**

```powershell
git add src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java
git add src/main/java/frc/robot/subsystems/shooter/ShooterIOSim.java
git add src/test/java/frc/robot/subsystems/shooter/ShooterIOSimTest.java
git commit -m "feat(shooter): add full physics simulation"
```

---

### Task 4: Runtime Mode Wiring

**Files:**

- Modify: `src/main/java/frc/robot/RobotContainer.java:20-47`
- Modify: `src/main/java/frc/robot/RobotContainer.java:101-103`
- Modify: `src/main/java/frc/robot/RobotContainer.java:179-186`
- Test: `src/test/java/frc/robot/RobotContainerShooterTest.java`

**Interfaces:**

- Consumes: `Shooter`, `ShooterIO`, `ShooterIOTalonFX`, `ShooterIOSim`, and `Constants.Mode`.
- Produces: retained `private final Shooter shooter` and package-private
  `static ShooterIO createShooterIO(Constants.Mode mode)`.

- [ ] **Step 1: Write failing mode-selection tests**

Create:

```java
package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants.Mode;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import org.junit.jupiter.api.Test;

class RobotContainerShooterTest {
  @Test
  void simulationUsesFullPhysicsIo() {
    assertInstanceOf(ShooterIOSim.class, RobotContainer.createShooterIO(Mode.SIM));
  }

  @Test
  void replayUsesNoOpIo() {
    assertInstanceOf(ShooterIO.NoOp.class, RobotContainer.createShooterIO(Mode.REPLAY));
  }
}
```

- [ ] **Step 2: Run mode-selection tests and verify RED**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.RobotContainerShooterTest
```

Expected: Java compilation fails because `createShooterIO` does not exist.

- [ ] **Step 3: Wire the shooter without behavior bindings**

Add imports for all four shooter types, add:

```java
private final Shooter shooter;
```

Immediately after hopper construction, add:

```java
shooter = new Shooter(createShooterIO(Constants.currentMode));
```

Add:

```java
static ShooterIO createShooterIO(Constants.Mode mode) {
  return switch (mode) {
    case REAL -> new ShooterIOTalonFX();
    case SIM -> new ShooterIOSim();
    case REPLAY -> new ShooterIO.NoOp();
  };
}
```

Do not modify `Superstructure.java`, button bindings, default commands, or autonomous registration.

- [ ] **Step 4: Run wiring and regression tests**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.RobotContainerShooterTest
.\gradlew.bat test --tests "frc.robot.*"
```

Expected: shooter factory tests and all existing robot-container tests pass.

- [ ] **Step 5: Commit runtime wiring**

```powershell
git add src/main/java/frc/robot/RobotContainer.java
git add src/test/java/frc/robot/RobotContainerShooterTest.java
git commit -m "feat(shooter): wire runtime IO modes"
```

---

### Task 5: Formatting, Full Verification, and Scope Audit

**Files:**

- Modify only shooter files or `RobotContainer.java` if Spotless changes formatting.

**Interfaces:**

- Consumes all previous tasks.
- Produces a clean, verified feature branch containing no Superstructure or binding changes.

- [ ] **Step 1: Apply formatting and inspect its scope**

Run:

```powershell
.\gradlew.bat spotlessApply
git status --short
git diff --stat
```

If Spotless modifies Java files, stage only:

```powershell
git add src/main/java/frc/robot/subsystems/shooter
git add src/test/java/frc/robot/subsystems/shooter
git add src/main/java/frc/robot/RobotContainer.java
git add src/test/java/frc/robot/RobotContainerShooterTest.java
```

- [ ] **Step 2: Run complete verification**

Run:

```powershell
.\gradlew.bat test
.\gradlew.bat testClasses
git diff --check
```

Expected:

- Every shooter and existing project test passes.
- `testClasses` succeeds.
- `git diff --check` prints no output.

- [ ] **Step 3: Audit final branch scope**

Run:

```powershell
git status --short
git diff --name-status HEAD~4..HEAD
git log --oneline --decorate -8
```

Confirm the branch changes only the approved shooter package, focused tests, `RobotContainer`,
the committed shooter design, and this local plan. Confirm `Superstructure.java`, hopper files,
controller bindings, and autonomous commands are unchanged.

- [ ] **Step 4: Commit formatter-only changes when present**

When Step 1 produced staged formatting changes:

```powershell
git commit -m "style(shooter): apply formatting"
```

When Step 1 produced no Java diff, skip this commit. Report the exact test counts and final commit
list before any push or pull-request action.
