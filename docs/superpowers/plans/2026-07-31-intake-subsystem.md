# Intake Subsystem Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an AdvantageKit-native intake subsystem with collision-conscious arm control, fused
CANcoder feedback, full simulation, and replay-safe runtime wiring before Superstructure exists.

**Architecture:** `Intake` owns the HOME/DEPLOYED/INTAKE/OUTTAKE/JUICER state machine and one-shot
state-entry commands. One `IntakeIO` boundary carries typed commands and structured telemetry for
both rollers, the arm TalonFX, and arm CANcoder; `IntakeIOTalonFX` and `IntakeIOSim` provide real and
full-physics implementations.

**Tech Stack:** Java 17, WPILib 2026.2.1, AdvantageKit `@AutoLog`, CTRE Phoenix 26.3.0, JUnit 5,
GradleRIO.

## Global Constraints

- Work from `feature/intake`, based on current `main`; preserve unrelated user changes.
- Use roller lead ID 21, opposed roller follower ID 20, arm TalonFX ID 10, and arm CANcoder ID 0.
- Use fused CANcoder feedback with clockwise-positive direction, `0.881064453125` magnet offset,
  `0.5` discontinuity point, 1:1 sensor-to-mechanism ratio, and 40:1 rotor-to-sensor ratio.
- Use `PositionVoltage` for arm position. Do not configure Motion Magic.
- Arm positions are deployed `0.00`, juicer pre-position `0.15`, juicer squeeze `0.25`, and stowed
  `0.37` mechanism rotations.
- Arm readiness is inclusive within `0.025` rotations.
- Use arm slot 0 for deploy/pre-juice (`kP = 14`) and slot 1 for stow/squeeze (`kP = 8`); both use
  `kV = 2.4`, `kG = 0.5`, arm-cosine gravity, and closed-loop feedforward sign.
- Use arm limits of 50 A stator, 30 A supply, and ±10 V; configure brake neutral.
- Use roller limits of 80 A stator, 40 A supply, 30 A lower supply after 0.2 seconds, and ±12 V;
  configure the lead clockwise-positive and both rollers coast neutral.
- Intake/outtake rollers use ±80 A `TorqueCurrentFOC` with 0.80 maximum absolute duty cycle.
- JUICER uses +80 A roller torque with 0.50 maximum absolute duty cycle.
- INTAKE alone uses -15 A arm tension; DEPLOYED and OUTTAKE use explicit brake `NeutralOut` with no
  position PID or torque hold.
- During DEPLOYING, autonomous uses -6 V roller output and all other modes use 0 V.
- Expose roller voltage, duty cycle, and torque current; do not add roller RPM control.
- Expose arm position, voltage, duty cycle, torque current, and brake-neutral controls.
- Voltage, duty-cycle, and position requests enable FOC. All requests update at 100 Hz.
- Torque-current requests use a 1 A deadband and coast-during-neutral override. Roller requests use
  caller-selected 0.80 or 0.50 maximum duty; arm requests use 1.0.
- Position, velocity, voltage, current, and CANcoder signals update at 50 Hz. Temperature and
  unspecified signals update at 4 Hz.
- Send mechanism commands only on state/juicer-phase entry; refresh and log inputs every loop.
- Current-state logging must match outputs issued during the same periodic loop.
- Brake-neutral states do not automatically redeploy after collision displacement.
- Simulation uses two Kraken X60 FOC roller models and one gravity-loaded Kraken X60 FOC arm model
  with 40:1 gearing, 13.370-inch length, 10 lb mass, `0.00–0.37` rotation limits, and `0.37` start.
- Add no bindings, default command, Superstructure behavior, named commands, fuel sensing, or jam
  handling.

---

## File Map

- Create `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`: hardware IDs, setpoints,
  gains, limits, Phoenix factories, timing, and simulation constants.
- Create `src/main/java/frc/robot/subsystems/intake/IntakeIO.java`: logged inputs and typed command
  boundary.
- Create `src/main/java/frc/robot/subsystems/intake/Intake.java`: state machine, direct controls,
  readiness, and AdvantageKit outputs.
- Create `src/main/java/frc/robot/subsystems/intake/IntakeIOTalonFX.java`: three TalonFX controllers,
  CANcoder, follower, requests, refresh, and CAN optimization.
- Create `src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java`: roller and arm physics.
- Modify `src/main/java/frc/robot/RobotContainer.java`: retained intake plus mode-specific factory.
- Create `src/test/java/frc/robot/subsystems/intake/IntakeTest.java`: state and direct-control tests.
- Create `src/test/java/frc/robot/subsystems/intake/IntakeLoggingTest.java`: real AdvantageKit output
  contract without starting the global logger lifecycle.
- Create `src/test/java/frc/robot/subsystems/intake/IntakeIOTalonFXConfigTest.java`: hardware config,
  request, refresh-group, and frequency tests without CAN hardware construction.
- Create `src/test/java/frc/robot/subsystems/intake/IntakeIOSimTest.java`: full simulation behavior.
- Create `src/test/java/frc/robot/RobotContainerIntakeTest.java`: REAL/SIM/REPLAY factory mapping.

---

### Task 1: Intake Contract and State Machine

**Files:**

- Create: `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`
- Create: `src/main/java/frc/robot/subsystems/intake/IntakeIO.java`
- Create: `src/main/java/frc/robot/subsystems/intake/Intake.java`
- Test: `src/test/java/frc/robot/subsystems/intake/IntakeTest.java`
- Test: `src/test/java/frc/robot/subsystems/intake/IntakeLoggingTest.java`

**Interfaces:**

- Produces `IntakeIO` methods `updateInputs(IntakeIOInputs)`, `setRollerVoltage(Voltage)`,
  `setRollerDutyCycle(double)`, `setRollerTorqueCurrent(Current, double)`,
  `setArmPosition(double, int)`, `setArmVoltage(Voltage)`, `setArmDutyCycle(double)`,
  `setArmTorqueCurrent(Current)`, and `setArmBrakeNeutral()`.
- Produces `Intake(IntakeIO)` and package-private `Intake(IntakeIO, BooleanSupplier)` for deterministic
  autonomous/teleop tests.
- Produces `IntakeState.HOME`, `INTAKE`, `OUTTAKE`, `DEPLOYED`, `DEPLOYING`, `STOWING`, `JUICER` and
  `JuicerPhase.PRE_JUICE`, `SQUEEZE`.
- Produces public state/phase/readiness accessors plus roller and arm direct-control methods.

- [ ] **Step 1: Load test-first rules before editing Java**

Read completely:

```text
C:\Users\dougd\.codex\plugins\cache\openai-curated-remote\superpowers\6.2.0\skills\test-driven-development\SKILL.md
C:\Users\dougd\.codex\plugins\cache\openai-curated-remote\superpowers\6.2.0\skills\test-driven-development\writing-good-tests.md
```

- [ ] **Step 2: Create the recording IO and failing startup/deployment tests**

Create `IntakeTest.java` with a fake that records every command independently:

```java
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.Intake.JuicerPhase;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class IntakeTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  private static final class RecordingIO implements IntakeIO {
    double measuredArmRotations;
    Double rollerVolts;
    Double rollerDutyCycle;
    Double rollerTorqueAmps;
    Double rollerMaxDuty;
    Double armPositionRotations;
    Integer armSlot;
    Double armVolts;
    Double armDutyCycle;
    Double armTorqueAmps;
    int armBrakeNeutralCalls;
    int armPositionCalls;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
      inputs.armPositionRotations = measuredArmRotations;
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
      rollerVolts = voltage.in(Volts);
    }

    @Override
    public void setRollerDutyCycle(double output) {
      rollerDutyCycle = output;
    }

    @Override
    public void setRollerTorqueCurrent(Current current, double maxAbsDutyCycle) {
      rollerTorqueAmps = current.in(Amps);
      rollerMaxDuty = maxAbsDutyCycle;
    }

    @Override
    public void setArmPosition(double rotations, int slot) {
      armPositionRotations = rotations;
      armSlot = slot;
      armPositionCalls++;
    }

    @Override
    public void setArmVoltage(Voltage voltage) {
      armVolts = voltage.in(Volts);
    }

    @Override
    public void setArmDutyCycle(double output) {
      armDutyCycle = output;
    }

    @Override
    public void setArmTorqueCurrent(Current current) {
      armTorqueAmps = current.in(Amps);
    }

    @Override
    public void setArmBrakeNeutral() {
      armBrakeNeutralCalls++;
    }
  }

  @Test
  void firstPeriodicExplicitlyCommandsSafeHome() {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.37;
    Intake intake = new Intake(io, () -> false);

    intake.periodic();

    assertEquals(0.0, io.rollerVolts, 1e-9);
    assertEquals(0.37, io.armPositionRotations, 1e-9);
    assertEquals(1, io.armSlot);
    assertEquals(IntakeState.HOME, intake.getCurrentState());
  }

  @Test
  void teleopDeployStopsRollerThenEntersBrakeNeutralDeployedState() {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.37;
    Intake intake = new Intake(io, () -> false);
    intake.periodic();
    intake.setDesiredState(IntakeState.DEPLOYED);
    io.measuredArmRotations = 0.20;

    intake.periodic();

    assertEquals(IntakeState.DEPLOYING, intake.getCurrentState());
    assertEquals(0.0, io.armPositionRotations, 1e-9);
    assertEquals(0, io.armSlot);
    assertEquals(0.0, io.rollerVolts, 1e-9);

    io.measuredArmRotations = 0.025;
    intake.periodic();

    assertEquals(IntakeState.DEPLOYED, intake.getCurrentState());
    assertEquals(1, io.armBrakeNeutralCalls);
    assertEquals(0.0, io.rollerVolts, 1e-9);
  }
}
```

- [ ] **Step 3: Add failing state-output, juicer, collision, validation, and clamp tests**

Add these exact behavior tests to `IntakeTest.java`:

```java
@Test
void autonomousDeployRunsRollerOutwardAtSixVolts() {
  RecordingIO io = new RecordingIO();
  io.measuredArmRotations = 0.37;
  Intake intake = new Intake(io, () -> true);
  intake.periodic();
  intake.setDesiredState(IntakeState.INTAKE);

  intake.periodic();

  assertEquals(IntakeState.DEPLOYING, intake.getCurrentState());
  assertEquals(-6.0, io.rollerVolts, 1e-9);
}

@Test
void disabledAutonomousDeploymentStopsRoller() {
  DriverStationSim.resetData();
  DriverStationSim.setAutonomous(true);
  DriverStationSim.setEnabled(false);
  DriverStationSim.notifyNewData();
  try {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.37;
    Intake intake = new Intake(io);
    intake.periodic();
    intake.setDesiredState(IntakeState.INTAKE);

    intake.periodic();

    assertEquals(IntakeState.DEPLOYING, intake.getCurrentState());
    assertEquals(0.0, io.rollerVolts, 1e-9);
  } finally {
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }
}

@Test
void deploymentReadinessAppliesIntakeOutputsBeforePublishingIntakeState() {
  RecordingIO io = new RecordingIO();
  io.measuredArmRotations = 0.37;
  Intake intake = new Intake(io, () -> false);
  intake.periodic();
  intake.setDesiredState(IntakeState.INTAKE);
  intake.periodic();

  io.measuredArmRotations = 0.025;
  intake.periodic();

  assertEquals(IntakeState.INTAKE, intake.getCurrentState());
  assertEquals(80.0, io.rollerTorqueAmps, 1e-9);
  assertEquals(0.80, io.rollerMaxDuty, 1e-9);
  assertEquals(-15.0, io.armTorqueAmps, 1e-9);
}

@Test
void desiredRequestDoesNotPublishTransitionBeforeItsOutputs() {
  RecordingIO io = new RecordingIO();
  io.measuredArmRotations = 0.37;
  Intake intake = new Intake(io, () -> false);
  intake.periodic();

  intake.setDesiredState(IntakeState.INTAKE);

  assertEquals(IntakeState.HOME, intake.getCurrentState());
  assertEquals(IntakeState.INTAKE, intake.getDesiredState());

  intake.periodic();
  assertEquals(IntakeState.DEPLOYING, intake.getCurrentState());
  assertEquals(0.0, io.armPositionRotations, 1e-9);
  assertEquals(0.0, io.rollerVolts, 1e-9);
}

@Test
void outtakeUsesBrakeNeutralAndNegativeRollerTorque() {
  RecordingIO io = new RecordingIO();
  io.measuredArmRotations = 0.0;
  Intake intake = new Intake(io, () -> false);
  intake.periodic();
  intake.setDesiredState(IntakeState.OUTTAKE);

  intake.periodic();

  assertEquals(IntakeState.OUTTAKE, intake.getCurrentState());
  assertEquals(-80.0, io.rollerTorqueAmps, 1e-9);
  assertEquals(0.80, io.rollerMaxDuty, 1e-9);
  assertEquals(1, io.armBrakeNeutralCalls);
  assertNull(io.armTorqueAmps);
}

@Test
void juicerUsesHalfDutyAndChangesToSlowSqueezeAtInclusiveBoundary() {
  RecordingIO io = new RecordingIO();
  io.measuredArmRotations = 0.0;
  Intake intake = new Intake(io, () -> false);
  intake.periodic();
  intake.setDesiredState(IntakeState.JUICER);

  intake.periodic();

  assertEquals(80.0, io.rollerTorqueAmps, 1e-9);
  assertEquals(0.50, io.rollerMaxDuty, 1e-9);
  assertEquals(0.15, io.armPositionRotations, 1e-9);
  assertEquals(0, io.armSlot);
  assertEquals(JuicerPhase.PRE_JUICE, intake.getJuicerPhase());

  io.measuredArmRotations = 0.125;
  intake.periodic();

  assertEquals(JuicerPhase.SQUEEZE, intake.getJuicerPhase());
  assertEquals(0.25, io.armPositionRotations, 1e-9);
  assertEquals(1, io.armSlot);
}

@Test
void brakeNeutralStateDoesNotAutomaticallyRedeployAfterCollision() {
  RecordingIO io = new RecordingIO();
  io.measuredArmRotations = 0.0;
  Intake intake = new Intake(io, () -> false);
  intake.periodic();
  intake.setDesiredState(IntakeState.DEPLOYED);
  intake.periodic();
  int positionCallsBeforeCollision = io.armPositionCalls;

  io.measuredArmRotations = 0.10;
  intake.periodic();

  assertEquals(IntakeState.DEPLOYED, intake.getCurrentState());
  assertEquals(positionCallsBeforeCollision, io.armPositionCalls);
}

@Test
void laterStateChangeRedeploysArmDisplacedByCollision() {
  RecordingIO io = new RecordingIO();
  io.measuredArmRotations = 0.0;
  Intake intake = new Intake(io, () -> false);
  intake.periodic();
  intake.setDesiredState(IntakeState.DEPLOYED);
  intake.periodic();
  io.measuredArmRotations = 0.10;
  int positionCallsBeforeNewRequest = io.armPositionCalls;

  intake.setDesiredState(IntakeState.OUTTAKE);
  intake.periodic();

  assertEquals(IntakeState.DEPLOYING, intake.getCurrentState());
  assertEquals(positionCallsBeforeNewRequest + 1, io.armPositionCalls);
  assertEquals(0.0, io.armPositionRotations, 1e-9);
  assertEquals(0, io.armSlot);
}

@Test
void homeRequestStowsWithSlowSlotAndLeavesHoldRequestActive() {
  RecordingIO io = new RecordingIO();
  io.measuredArmRotations = 0.0;
  Intake intake = new Intake(io, () -> false);
  intake.periodic();
  intake.setDesiredState(IntakeState.DEPLOYED);
  intake.periodic();

  intake.setDesiredState(IntakeState.HOME);
  intake.periodic();
  int positionCallsBeforeReady = io.armPositionCalls;
  assertEquals(IntakeState.STOWING, intake.getCurrentState());
  assertEquals(0.37, io.armPositionRotations, 1e-9);
  assertEquals(1, io.armSlot);
  assertEquals(0.0, io.rollerVolts, 1e-9);

  io.measuredArmRotations = 0.345;
  intake.periodic();
  assertEquals(IntakeState.HOME, intake.getCurrentState());
  assertEquals(positionCallsBeforeReady, io.armPositionCalls);
  assertEquals(0.37, io.armPositionRotations, 1e-9);
}

@Test
void repeatedRequestForActiveStateDoesNotResendOutputs() {
  RecordingIO io = new RecordingIO();
  io.measuredArmRotations = 0.0;
  Intake intake = new Intake(io, () -> false);
  intake.periodic();
  intake.setDesiredState(IntakeState.DEPLOYED);
  intake.periodic();
  int brakeCalls = io.armBrakeNeutralCalls;

  intake.setDesiredState(IntakeState.DEPLOYED);
  intake.periodic();

  assertEquals(brakeCalls, io.armBrakeNeutralCalls);
}

@Test
void readinessUsesInclusivePointZeroTwoFiveTolerance() {
  RecordingIO io = new RecordingIO();
  Intake intake = new Intake(io, () -> false);

  io.measuredArmRotations = 0.025;
  intake.periodic();
  assertTrue(intake.isArmAtDeployed());
  io.measuredArmRotations = 0.02501;
  intake.periodic();
  assertFalse(intake.isArmAtDeployed());
  io.measuredArmRotations = 0.345;
  intake.periodic();
  assertTrue(intake.isArmAtStowed());
  io.measuredArmRotations = 0.12499;
  intake.periodic();
  assertFalse(intake.isArmAtPreJuice());
}

@Test
void invalidDesiredStatesAreRejectedWithoutCorruptingDesiredState() {
  Intake intake = new Intake(new RecordingIO(), () -> false);

  assertThrows(NullPointerException.class, () -> intake.setDesiredState(null));
  assertThrows(
      IllegalArgumentException.class, () -> intake.setDesiredState(IntakeState.DEPLOYING));
  assertThrows(
      IllegalArgumentException.class, () -> intake.setDesiredState(IntakeState.STOWING));
  assertEquals(IntakeState.HOME, intake.getDesiredState());
}

@Test
void directControlsClampAtSubsystemBoundary() {
  RecordingIO io = new RecordingIO();
  Intake intake = new Intake(io, () -> false);

  intake.runRollerVoltage(Volts.of(20.0));
  intake.runRollerPercentage(-2.0);
  intake.runRollerTorqueCurrentFOC(Amps.of(100.0), 1.25);
  intake.runArmVoltage(Volts.of(-20.0));
  intake.runArmPercentage(2.0);
  intake.runArmTorqueCurrentFOC(Amps.of(-75.0));
  intake.runArmPosition(0.80, 9);
  intake.brakeArm();

  assertEquals(12.0, io.rollerVolts, 1e-9);
  assertEquals(-1.0, io.rollerDutyCycle, 1e-9);
  assertEquals(80.0, io.rollerTorqueAmps, 1e-9);
  assertEquals(1.0, io.rollerMaxDuty, 1e-9);
  assertEquals(-10.0, io.armVolts, 1e-9);
  assertEquals(1.0, io.armDutyCycle, 1e-9);
  assertEquals(-50.0, io.armTorqueAmps, 1e-9);
  assertEquals(0.37, io.armPositionRotations, 1e-9);
  assertEquals(1, io.armSlot);
  assertEquals(1, io.armBrakeNeutralCalls);
}
```

- [ ] **Step 4: Run state tests and verify RED**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.intake.IntakeTest --console=plain
```

Expected: test compilation fails because `Intake`, `IntakeIO`, and generated
`IntakeIOInputsAutoLogged` do not exist.

- [ ] **Step 5: Add exact constants and IO schema**

Create `IntakeConstants.java` with typed values copied from Global Constraints. Include these
names so later tasks consume one stable contract:

```java
static final int ROLLER_LEAD_ID = 21;
static final int ROLLER_FOLLOWER_ID = 20;
static final int ARM_MOTOR_ID = 10;
static final int ARM_CANCODER_ID = 0;
static final int ARM_FAST_SLOT = 0;
static final int ARM_SLOW_SLOT = 1;
static final double ARM_DEPLOYED_ROTATIONS = 0.0;
static final double ARM_PRE_JUICE_ROTATIONS = 0.15;
static final double ARM_SQUEEZE_ROTATIONS = 0.25;
static final double ARM_STOWED_ROTATIONS = 0.37;
static final double ARM_POSITION_TOLERANCE = 0.025;
static final Current ROLLER_STATOR_LIMIT = Amps.of(80.0);
static final Current ARM_STATOR_LIMIT = Amps.of(50.0);
static final Current INTAKE_ROLLER_CURRENT = Amps.of(80.0);
static final Current OUTTAKE_ROLLER_CURRENT = Amps.of(-80.0);
static final Current ARM_INTAKE_TENSION_CURRENT = Amps.of(-15.0);
static final double ROLLER_STATE_MAX_DUTY = 0.80;
static final double ROLLER_JUICER_MAX_DUTY = 0.50;
static final Voltage ROLLER_MAX_VOLTAGE = Volts.of(12.0);
static final Voltage ARM_MAX_VOLTAGE = Volts.of(10.0);
static final Voltage AUTONOMOUS_DEPLOY_ROLLER_VOLTAGE = Volts.of(-6.0);
static final double CONTROL_HZ = 100.0;
static final double MECHANISM_HZ = 50.0;
static final double SLOW_HZ = 4.0;
```

Create `IntakeIO.java`:

```java
public interface IntakeIO {
  final class NoOp implements IntakeIO {}

  @AutoLog
  class IntakeIOInputs {
    public boolean rollerLeadConnected;
    public double rollerLeadPositionRotations;
    public double rollerLeadVelocityRpm;
    public double rollerLeadAppliedVolts;
    public double rollerLeadCurrentAmps;
    public double rollerLeadTempCelsius;
    public boolean rollerFollowerConnected;
    public double rollerFollowerPositionRotations;
    public double rollerFollowerVelocityRpm;
    public double rollerFollowerAppliedVolts;
    public double rollerFollowerCurrentAmps;
    public double rollerFollowerTempCelsius;
    public boolean armConnected;
    public double armPositionRotations;
    public double armVelocityRpm;
    public double armAppliedVolts;
    public double armCurrentAmps;
    public double armTempCelsius;
    public boolean armCancoderConnected;
    public double armCancoderPositionRotations;
    public double armCancoderAbsolutePositionRotations;
    public double armCancoderVelocityRpm;
  }

  default void updateInputs(IntakeIOInputs inputs) {}
  default void setRollerVoltage(Voltage voltage) {}
  default void setRollerDutyCycle(double output) {}
  default void setRollerTorqueCurrent(Current current, double maxAbsDutyCycle) {}
  default void setArmPosition(double rotations, int slot) {}
  default void setArmVoltage(Voltage voltage) {}
  default void setArmDutyCycle(double output) {}
  default void setArmTorqueCurrent(Current current) {}
  default void setArmBrakeNeutral() {}
}
```

- [ ] **Step 6: Implement direct controls, readiness, and state selection**

Create `Intake.java`. Public construction delegates to the testable autonomous supplier:

```java
public Intake(IntakeIO io) {
  this(io, DriverStation::isAutonomousEnabled);
}

Intake(IntakeIO io, BooleanSupplier autonomousSupplier) {
  this.io = Objects.requireNonNull(io);
  this.autonomousSupplier = Objects.requireNonNull(autonomousSupplier);
}
```

Initialize desired `HOME`, current `STOWING`, and juicer phase `PRE_JUICE`. Implement public direct
methods for all IO controls. Clamp roller voltage to ±12 V, roller current to ±80 A, arm voltage to
±10 V, arm current to ±50 A, percentages to ±1, roller maximum duty to `[0, 1]`, arm positions to
`[0.00, 0.37]`, and arm slots to `[0, 1]` before forwarding. Implement readiness with inclusive
comparison:

```java
private boolean isArmAt(double targetRotations) {
  return Math.abs(targetRotations - inputs.armPositionRotations) <= ARM_POSITION_TOLERANCE;
}
```

Validate desired states before assignment. The setter changes intent only; it must not publish a
transition state before the matching outputs are issued in `periodic()`:

```java
public void setDesiredState(IntakeState state) {
  Objects.requireNonNull(state);
  if (state == IntakeState.DEPLOYING || state == IntakeState.STOWING) {
    throw new IllegalArgumentException("Transition states cannot be desired");
  }
  desiredState = state;
}
```

- [ ] **Step 7: Implement entry-driven handlers with same-loop steady outputs**

Call `io.updateInputs`, `Logger.processInputs`, desired-state reconciliation, state handling, then
output logging from `periodic()`. Track the last reconciled desired state separately from
`lastCommandedState`; only a genuinely new desired state selects a transition:

```java
private void reconcileDesiredState() {
  if (desiredState == lastReconciledDesiredState) return;
  lastReconciledDesiredState = desiredState;
  lastCommandedState = null;
  switch (desiredState) {
    case HOME -> currentState = IntakeState.STOWING;
    case INTAKE, OUTTAKE, DEPLOYED -> currentState = IntakeState.DEPLOYING;
    case JUICER -> {
      currentState = IntakeState.JUICER;
      juicerPhase = JuicerPhase.PRE_JUICE;
      lastJuicerPhase = null;
    }
    case DEPLOYING, STOWING -> throw new IllegalStateException("Desired state was validated");
  }
}
```

Initialize `lastReconciledDesiredState` to `HOME` because startup already begins in `STOWING`. Use
`lastCommandedState` and `lastJuicerPhase` to suppress repeated CAN commands.

In `handleDeployingState`, check readiness before issuing a new transition request. This allows
already-deployed arms to switch steady outputs without a redundant position request:

```java
private void handleDeployingState() {
  if (isArmAtDeployed()) {
    enterDeployedTargetState();
    return;
  }
  if (!isStateEntry()) return;
  deployArm();
  runRollerVoltage(
      autonomousSupplier.getAsBoolean() ? AUTONOMOUS_DEPLOY_ROLLER_VOLTAGE : Volts.zero());
  markStateEntryHandled();
}

private void enterDeployedTargetState() {
  IntakeState nextState = switch (desiredState) {
    case INTAKE -> IntakeState.INTAKE;
    case OUTTAKE -> IntakeState.OUTTAKE;
    default -> IntakeState.DEPLOYED;
  };
  switch (nextState) {
    case INTAKE -> commandIntakeOutputs();
    case OUTTAKE -> commandOuttakeOutputs();
    case DEPLOYED -> commandDeployedOutputs();
    default -> throw new IllegalStateException("Unexpected deployed target " + nextState);
  }
  currentState = nextState;
  lastCommandedState = nextState;
}
```

The command helpers above issue the exact steady outputs. The ordinary steady-state handlers call
the same helpers only on entry:

```java
private void commandIntakeOutputs() {
  runArmTorqueCurrentFOC(ARM_INTAKE_TENSION_CURRENT);
  runRollerTorqueCurrentFOC(INTAKE_ROLLER_CURRENT, ROLLER_STATE_MAX_DUTY);
}

private void commandOuttakeOutputs() {
  brakeArm();
  runRollerTorqueCurrentFOC(OUTTAKE_ROLLER_CURRENT, ROLLER_STATE_MAX_DUTY);
}

private void commandDeployedOutputs() {
  brakeArm();
  stopRoller();
}

private void handleIntakeState() {
  if (!isStateEntry()) return;
  commandIntakeOutputs();
  markStateEntryHandled();
}

private void handleOuttakeState() {
  if (!isStateEntry()) return;
  commandOuttakeOutputs();
  markStateEntryHandled();
}

private void handleDeployedState() {
  if (!isStateEntry()) return;
  commandDeployedOutputs();
  markStateEntryHandled();
}
```

STOWING first sends stow slot 1 plus roller 0 V on entry, then checks readiness. When ready, set
`currentState = HOME` and `lastCommandedState = HOME` without changing the active position request.
This ordering guarantees safe outputs on the first periodic call even when the arm starts stowed.
JUICER sends roller +80 A/0.50 once, commands pre-juice slot 0, and when ready commands squeeze slot
1 before publishing `SQUEEZE`.

- [ ] **Step 8: Add the failing AdvantageKit output contract test**

Create `IntakeLoggingTest.java` using an isolated `LogTable`. Do not call `Logger.start()` or
`Logger.end()` because AdvantageKit owns a single non-restartable receiver thread:

```java
@Test
void periodicPublishesMeasuredArmAndStateOutputs() throws ReflectiveOperationException {
  Field runningField = Logger.class.getDeclaredField("running");
  Field entryField = Logger.class.getDeclaredField("entry");
  Field outputTableField = Logger.class.getDeclaredField("outputTable");
  runningField.setAccessible(true);
  entryField.setAccessible(true);
  outputTableField.setAccessible(true);
  boolean previousRunning = runningField.getBoolean(null);
  LogTable previousEntry = (LogTable) entryField.get(null);
  LogTable previousOutput = (LogTable) outputTableField.get(null);
  LogTable testEntry = new LogTable(0);
  runningField.setBoolean(null, true);
  entryField.set(null, testEntry);
  outputTableField.set(null, testEntry.getSubtable("RealOutputs"));
  try {
    Intake intake =
        new Intake(
            new IntakeIO() {
              @Override
              public void updateInputs(IntakeIOInputs inputs) {
                inputs.armPositionRotations = 0.37;
              }
            },
            () -> false);
    intake.periodic();
    assertEquals(0.37, testEntry.get("RealOutputs/Intake/MeasuredArmRotations", Double.NaN), 1e-9);
    assertEquals("HOME", testEntry.get("RealOutputs/Intake/CurrentState", ""));
  } finally {
    outputTableField.set(null, previousOutput);
    entryField.set(null, previousEntry);
    runningField.setBoolean(null, previousRunning);
  }
}
```

- [ ] **Step 9: Verify logging RED, add exact outputs, then verify GREEN**

Run the logging test before adding output calls. Expected: FAIL because both output keys are absent.
Then add:

```java
Logger.recordOutput("Intake/DesiredState", desiredState.name());
Logger.recordOutput("Intake/CurrentState", currentState.name());
Logger.recordOutput("Intake/JuicerPhase", juicerPhase.name());
Logger.recordOutput("Intake/MeasuredArmRotations", inputs.armPositionRotations);
Logger.recordOutput("Intake/ArmAtDeployed", isArmAtDeployed());
Logger.recordOutput("Intake/ArmAtStowed", isArmAtStowed());
Logger.recordOutput("Intake/ArmAtPreJuice", isArmAtPreJuice());
```

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.intake.IntakeLoggingTest --console=plain
.\gradlew.bat test --tests frc.robot.subsystems.intake.IntakeTest --console=plain
```

Expected: both focused test classes pass.

- [ ] **Step 10: Commit the state and IO contract**

```powershell
git add src/main/java/frc/robot/subsystems/intake/IntakeConstants.java
git add src/main/java/frc/robot/subsystems/intake/IntakeIO.java
git add src/main/java/frc/robot/subsystems/intake/Intake.java
git add src/test/java/frc/robot/subsystems/intake/IntakeTest.java
git add src/test/java/frc/robot/subsystems/intake/IntakeLoggingTest.java
git commit -m "feat(intake): add state and IO contract"
```

---

### Task 2: TalonFX and CANcoder Hardware IO

**Files:**

- Modify: `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`
- Create: `src/main/java/frc/robot/subsystems/intake/IntakeIOTalonFX.java`
- Test: `src/test/java/frc/robot/subsystems/intake/IntakeIOTalonFXConfigTest.java`

**Interfaces:**

- Consumes all `IntakeIO` methods and constants from Task 1.
- Produces `IntakeIOTalonFX`, package-private configuration/request factories, refresh-group
  factories, and `StatusFrequencyConfig` for tests and RobotContainer.

- [ ] **Step 1: Write failing hardware configuration and request tests**

Create `IntakeIOTalonFXConfigTest.java` with these assertions:

```java
@Test
void rollerConfigsPreserveIdsDirectionLimitsAndFollowerAlignment() {
  var lead = IntakeConstants.createRollerLeadConfig();
  var follower = IntakeConstants.createRollerFollowerConfig();
  Follower request = IntakeIOTalonFX.createRollerFollowerRequest();

  assertEquals(21, IntakeConstants.ROLLER_LEAD_ID);
  assertEquals(20, IntakeConstants.ROLLER_FOLLOWER_ID);
  assertEquals(InvertedValue.Clockwise_Positive, lead.MotorOutput.Inverted);
  assertEquals(NeutralModeValue.Coast, lead.MotorOutput.NeutralMode);
  assertEquals(NeutralModeValue.Coast, follower.MotorOutput.NeutralMode);
  assertEquals(80.0, lead.CurrentLimits.StatorCurrentLimit, 1e-9);
  assertTrue(lead.CurrentLimits.StatorCurrentLimitEnable);
  assertEquals(40.0, lead.CurrentLimits.SupplyCurrentLimit, 1e-9);
  assertTrue(lead.CurrentLimits.SupplyCurrentLimitEnable);
  assertEquals(30.0, lead.CurrentLimits.SupplyCurrentLowerLimit, 1e-9);
  assertEquals(0.2, lead.CurrentLimits.SupplyCurrentLowerTime, 1e-9);
  assertEquals(12.0, lead.Voltage.PeakForwardVoltage, 1e-9);
  assertEquals(-12.0, lead.Voltage.PeakReverseVoltage, 1e-9);
  assertTrue(follower.CurrentLimits.StatorCurrentLimitEnable);
  assertEquals(80.0, follower.CurrentLimits.StatorCurrentLimit, 1e-9);
  assertTrue(follower.CurrentLimits.SupplyCurrentLimitEnable);
  assertEquals(40.0, follower.CurrentLimits.SupplyCurrentLimit, 1e-9);
  assertEquals(30.0, follower.CurrentLimits.SupplyCurrentLowerLimit, 1e-9);
  assertEquals(0.2, follower.CurrentLimits.SupplyCurrentLowerTime, 1e-9);
  assertEquals(12.0, follower.Voltage.PeakForwardVoltage, 1e-9);
  assertEquals(-12.0, follower.Voltage.PeakReverseVoltage, 1e-9);
  assertEquals(21, request.LeaderID);
  assertEquals(MotorAlignmentValue.Opposed, request.MotorAlignment);
  assertEquals(100.0, request.UpdateFreqHz, 1e-9);
}

@Test
void armAndCancoderConfigsUseApprovedFusedFeedbackAndGains() {
  var arm = IntakeConstants.createArmConfig();
  var cancoder = IntakeConstants.createArmCancoderConfig();

  assertEquals(10, IntakeConstants.ARM_MOTOR_ID);
  assertEquals(0, IntakeConstants.ARM_CANCODER_ID);
  assertEquals(50.0, arm.CurrentLimits.StatorCurrentLimit, 1e-9);
  assertTrue(arm.CurrentLimits.StatorCurrentLimitEnable);
  assertEquals(30.0, arm.CurrentLimits.SupplyCurrentLimit, 1e-9);
  assertTrue(arm.CurrentLimits.SupplyCurrentLimitEnable);
  assertEquals(10.0, arm.Voltage.PeakForwardVoltage, 1e-9);
  assertEquals(-10.0, arm.Voltage.PeakReverseVoltage, 1e-9);
  assertEquals(NeutralModeValue.Brake, arm.MotorOutput.NeutralMode);
  assertEquals(InvertedValue.Clockwise_Positive, arm.MotorOutput.Inverted);
  assertEquals(0, arm.Feedback.FeedbackRemoteSensorID);
  assertEquals(FeedbackSensorSourceValue.FusedCANcoder, arm.Feedback.FeedbackSensorSource);
  assertEquals(1.0, arm.Feedback.SensorToMechanismRatio, 1e-9);
  assertEquals(40.0, arm.Feedback.RotorToSensorRatio, 1e-9);
  assertEquals(14.0, arm.Slot0.kP, 1e-9);
  assertEquals(8.0, arm.Slot1.kP, 1e-9);
  assertEquals(2.4, arm.Slot0.kV, 1e-9);
  assertEquals(0.5, arm.Slot1.kG, 1e-9);
  assertEquals(GravityTypeValue.Arm_Cosine, arm.Slot0.GravityType);
  assertEquals(
      StaticFeedforwardSignValue.UseClosedLoopSign, arm.Slot1.StaticFeedforwardSign);
  assertEquals(0.881064453125, cancoder.MagnetSensor.MagnetOffset, 1e-12);
  assertEquals(0.5, cancoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint, 1e-9);
  assertEquals(
      SensorDirectionValue.Clockwise_Positive, cancoder.MagnetSensor.SensorDirection);
}

@Test
void requestsUseApprovedFocDutyDeadbandNeutralAndFrequencySettings() {
  VoltageOut voltage = IntakeIOTalonFX.createVoltageRequest();
  DutyCycleOut duty = IntakeIOTalonFX.createDutyCycleRequest();
  TorqueCurrentFOC rollerTorque = IntakeIOTalonFX.createRollerTorqueRequest();
  TorqueCurrentFOC armTorque = IntakeIOTalonFX.createArmTorqueRequest();
  PositionVoltage position = IntakeIOTalonFX.createArmPositionRequest();
  NeutralOut neutral = IntakeIOTalonFX.createArmNeutralRequest();

  assertTrue(voltage.EnableFOC);
  assertTrue(duty.EnableFOC);
  assertTrue(position.EnableFOC);
  assertEquals(1.0, rollerTorque.Deadband, 1e-9);
  assertEquals(0.80, rollerTorque.MaxAbsDutyCycle, 1e-9);
  assertTrue(rollerTorque.OverrideCoastDurNeutral);
  assertEquals(1.0, armTorque.Deadband, 1e-9);
  assertEquals(1.0, armTorque.MaxAbsDutyCycle, 1e-9);
  assertTrue(armTorque.OverrideCoastDurNeutral);
  assertEquals(100.0, voltage.UpdateFreqHz, 1e-9);
  assertEquals(100.0, duty.UpdateFreqHz, 1e-9);
  assertEquals(100.0, rollerTorque.UpdateFreqHz, 1e-9);
  assertEquals(100.0, armTorque.UpdateFreqHz, 1e-9);
  assertEquals(100.0, position.UpdateFreqHz, 1e-9);
  assertEquals(100.0, neutral.UpdateFreqHz, 1e-9);
}

@Test
void statusFrequenciesUseApprovedCanSchedule() {
  var frequencies = IntakeIOTalonFX.createStatusFrequencyConfig();
  assertEquals(50.0, frequencies.mechanismHz(), 1e-9);
  assertEquals(4.0, frequencies.temperatureHz(), 1e-9);
  assertEquals(4.0, frequencies.unspecifiedHz(), 1e-9);
}
```

Also test `createTemperatureRefreshSignals(...)` returns all three motor-temperature signals and
`createCancoderRefreshSignals(...)` returns position, absolute position, and velocity in order.

- [ ] **Step 2: Run hardware tests and verify RED**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.intake.IntakeIOTalonFXConfigTest --console=plain
```

Expected: compilation fails because configuration and request factories do not exist.

- [ ] **Step 3: Add exact TalonFX and CANcoder configuration factories**

In `IntakeConstants`, build fresh configs per call. Roller configs apply identical current and
voltage safety groups to lead and follower, with stator and supply limit enable bits explicitly
`true`, plus their motor-output groups. Arm config likewise explicitly enables both current limits,
then applies voltage, motor output, slot 0, slot 1, and feedback:

```java
new FeedbackConfigs()
    .withFeedbackRemoteSensorID(ARM_CANCODER_ID)
    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
    .withSensorToMechanismRatio(1.0)
    .withRotorToSensorRatio(40.0)
```

Build CANcoder config with:

```java
new CANcoderConfiguration()
    .withMagnetSensor(
        new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(0.5)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withMagnetOffset(0.881064453125))
```

Do not instantiate `CoreCANcoder` inside constants and do not configure Motion Magic.

- [ ] **Step 4: Implement request factories and hardware ownership**

Create `IntakeIOTalonFX.java` owning three `TalonFX` objects and one `CANcoder`. Create independent
request instances for roller and arm commands. Factories are:

```java
static VoltageOut createVoltageRequest() {
  return new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(CONTROL_HZ);
}

static DutyCycleOut createDutyCycleRequest() {
  return new DutyCycleOut(0.0).withEnableFOC(true).withUpdateFreqHz(CONTROL_HZ);
}

static TorqueCurrentFOC createRollerTorqueRequest() {
  return new TorqueCurrentFOC(0.0)
      .withMaxAbsDutyCycle(ROLLER_STATE_MAX_DUTY)
      .withDeadband(Amps.of(1.0))
      .withOverrideCoastDurNeutral(true)
      .withUpdateFreqHz(CONTROL_HZ);
}

static TorqueCurrentFOC createArmTorqueRequest() {
  return new TorqueCurrentFOC(0.0)
      .withMaxAbsDutyCycle(1.0)
      .withDeadband(Amps.of(1.0))
      .withOverrideCoastDurNeutral(true)
      .withUpdateFreqHz(CONTROL_HZ);
}

static PositionVoltage createArmPositionRequest() {
  return new PositionVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(CONTROL_HZ);
}

static NeutralOut createArmNeutralRequest() {
  return new NeutralOut().withUpdateFreqHz(CONTROL_HZ);
}

static Follower createRollerFollowerRequest() {
  return new Follower(ROLLER_LEAD_ID, MotorAlignmentValue.Opposed)
      .withUpdateFreqHz(CONTROL_HZ);
}
```

- [ ] **Step 5: Configure devices, signals, optimization, and follower**

Constructor order:

1. Apply CANcoder config with `tryUntilOk(5, () -> cancoder.getConfigurator().apply(..., 0.25))`.
2. Apply roller lead, roller follower, and arm configs through the same bounded retry helper.
3. Clear sticky faults on all four devices with bounded retries.
4. Set all motor position/velocity/voltage/current signals and all three CANcoder signals to 50 Hz.
5. Set all three motor temperatures to 4 Hz.
6. Call `ParentDevice.optimizeBusUtilizationForAll(4.0, lead, follower, arm, cancoder)`.
7. Apply the opposed follower request with bounded retry.

Use one falling-edge 0.5-second connection debouncer per device.

- [ ] **Step 6: Implement refresh and output methods**

In `updateInputs`, refresh four mechanism signals per motor, all three temperatures, and all three
CANcoder signals before reading. Convert rotations/second to RPM by multiplying by 60. Derive each
connection flag from its mechanism refresh status and debouncer. Populate all 22 input fields every
cycle.

Output methods use exact clamps and requests:

```java
rollerLead.setControl(
    rollerTorqueRequest
        .withOutput(MathUtil.clamp(current.in(Amps), -80.0, 80.0))
        .withMaxAbsDutyCycle(MathUtil.clamp(maxAbsDutyCycle, 0.0, 1.0)));

arm.setControl(
    armPositionRequest
        .withPosition(
            MathUtil.clamp(rotations, ARM_DEPLOYED_ROTATIONS, ARM_STOWED_ROTATIONS))
        .withSlot(MathUtil.clamp(slot, ARM_FAST_SLOT, ARM_SLOW_SLOT)));

arm.setControl(armNeutralRequest);
```

Voltage, duty, and arm torque methods use their mechanism-specific ±12 V, ±10 V, ±80 A, ±50 A,
and ±1 clamps.

- [ ] **Step 7: Run hardware and subsystem regression tests**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.intake.IntakeIOTalonFXConfigTest --console=plain
.\gradlew.bat test --tests "frc.robot.subsystems.intake.*" --console=plain
```

Expected: all Task 1 and hardware tests pass without constructing CAN hardware.

- [ ] **Step 8: Commit hardware IO**

```powershell
git add src/main/java/frc/robot/subsystems/intake/IntakeConstants.java
git add src/main/java/frc/robot/subsystems/intake/IntakeIOTalonFX.java
git add src/test/java/frc/robot/subsystems/intake/IntakeIOTalonFXConfigTest.java
git commit -m "feat(intake): add TalonFX and CANcoder IO"
```

---

### Task 3: Full Intake Physics Simulation

**Files:**

- Modify: `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`
- Create: `src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java`
- Test: `src/test/java/frc/robot/subsystems/intake/IntakeIOSimTest.java`

**Interfaces:**

- Consumes every `IntakeIO` command from Task 1.
- Produces dynamic telemetry for both opposed rollers, the arm motor, and its simulated CANcoder.
- Owns no state-machine policy; it only simulates the most recently selected IO control mode.

- [ ] **Step 1: Write failing initialization and complete-telemetry tests**

Create `IntakeIOSimTest.java`, initialize HAL once, and restore roboRIO voltage around every test:

```java
private double originalBatteryVoltage;

@BeforeAll
static void initializeHal() {
  HAL.initialize(500, 0);
}

@BeforeEach
void resetBattery() {
  originalBatteryVoltage = RoboRioSim.getVInVoltage();
  RoboRioSim.setVInVoltage(12.0);
}

@AfterEach
void restoreBattery() {
  RoboRioSim.setVInVoltage(originalBatteryVoltage);
}

@Test
void startsStowedAndOverwritesEveryTelemetryField() {
  IntakeIOSim io = new IntakeIOSim();
  IntakeIO.IntakeIOInputs inputs = inputsWithDistinctSentinels();

  io.updateInputs(inputs);

  assertTrue(inputs.rollerLeadConnected);
  assertTrue(inputs.rollerFollowerConnected);
  assertTrue(inputs.armConnected);
  assertTrue(inputs.armCancoderConnected);
  assertEquals(0.37, inputs.armPositionRotations, 1e-6);
  assertEquals(inputs.armPositionRotations, inputs.armCancoderPositionRotations, 1e-9);
  assertEquals(0.0, inputs.rollerLeadTempCelsius, 1e-9);
  assertEquals(0.0, inputs.rollerFollowerTempCelsius, 1e-9);
  assertEquals(0.0, inputs.armTempCelsius, 1e-9);
  assertNoSentinelValuesRemain(inputs);
}
```

The two helpers must assign and then check unique nonphysical values for all 22 fields, including
all connection and temperature fields. This guards against the stale/default telemetry defects
previously found in hopper simulation.

- [ ] **Step 2: Add failing arm position, brake, and hard-stop tests**

Add:

```java
@Test
void positionControlReachesDeployJuicerAndStowUsingRequestedSlots() {
  IntakeIOSim io = new IntakeIOSim();
  IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
  io.setArmPosition(0.0, IntakeConstants.ARM_FAST_SLOT);
  update(io, inputs, 300);
  assertEquals(0.0, inputs.armPositionRotations, 0.025);

  io.setArmPosition(0.15, IntakeConstants.ARM_FAST_SLOT);
  update(io, inputs, 300);
  assertEquals(0.15, inputs.armPositionRotations, 0.025);

  io.setArmPosition(0.25, IntakeConstants.ARM_SLOW_SLOT);
  update(io, inputs, 300);
  assertEquals(0.25, inputs.armPositionRotations, 0.025);

  io.setArmPosition(0.37, IntakeConstants.ARM_SLOW_SLOT);
  update(io, inputs, 400);
  assertEquals(0.37, inputs.armPositionRotations, 0.025);
}

@Test
void brakeNeutralStopsDrivingPreviousPositionTarget() {
  IntakeIOSim io = new IntakeIOSim();
  IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
  io.setArmPosition(0.15, IntakeConstants.ARM_FAST_SLOT);
  update(io, inputs, 25);

  io.setArmBrakeNeutral();
  io.updateInputs(inputs);

  assertEquals(0.0, inputs.armAppliedVolts, 1e-9);
}

@Test
void physicalArmCannotPassDeployedOrStowedStops() {
  IntakeIOSim io = new IntakeIOSim();
  IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
  io.setArmVoltage(Volts.of(10.0));
  update(io, inputs, 200);
  assertTrue(inputs.armPositionRotations <= 0.37 + 1e-6);

  io.setArmVoltage(Volts.of(-10.0));
  update(io, inputs, 400);
  assertTrue(inputs.armPositionRotations >= -1e-6);
}
```

The `update` helper loops `io.updateInputs(inputs)` exactly the requested number of 20 ms cycles.

- [ ] **Step 3: Add failing roller modes, arm direct modes, and battery tests**

Test each exposed mode independently:

```java
@Test
void rollerVoltageDutyAndTorqueModesProduceOpposedDynamicTelemetry() {
  IntakeIOSim io = new IntakeIOSim();
  IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();

  io.setRollerVoltage(Volts.of(-1.0));
  io.updateInputs(inputs);
  assertEquals(-1.0, inputs.rollerLeadAppliedVolts, 1e-9);
  assertEquals(1.0, inputs.rollerFollowerAppliedVolts, 1e-9);
  assertTrue(inputs.rollerLeadVelocityRpm < 0.0);
  assertTrue(inputs.rollerFollowerVelocityRpm > 0.0);

  RoboRioSim.setVInVoltage(12.0);
  io = new IntakeIOSim();
  io.setRollerDutyCycle(0.5);
  io.updateInputs(inputs);
  assertTrue(inputs.rollerLeadAppliedVolts > 0.0);
  assertEquals(-inputs.rollerLeadAppliedVolts, inputs.rollerFollowerAppliedVolts, 1e-9);
  assertTrue(inputs.rollerLeadCurrentAmps <= 80.0 + 1e-6);
  assertTrue(inputs.rollerFollowerCurrentAmps <= 80.0 + 1e-6);

  RoboRioSim.setVInVoltage(12.0);
  io = new IntakeIOSim();
  io.setRollerVoltage(Volts.of(4.0));
  update(io, inputs, 200);
  RoboRioSim.setVInVoltage(12.0);
  io.setRollerTorqueCurrent(Amps.of(80.0), 0.50);
  io.updateInputs(inputs);
  assertTrue(inputs.rollerLeadAppliedVolts > 0.0);
  assertTrue(
      Math.abs(inputs.rollerLeadAppliedVolts)
          <= 0.50 * RoboRioSim.getVInVoltage() + 1e-9);
  assertTrue(inputs.rollerLeadVelocityRpm > 0.0);
  assertTrue(inputs.rollerFollowerVelocityRpm < 0.0);
}

@Test
void armVoltageDutyAndTorqueModesMoveThePhysicalArm() {
  IntakeIOSim io = new IntakeIOSim();
  IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
  io.setArmVoltage(Volts.of(-4.0));
  update(io, inputs, 20);
  double afterVoltage = inputs.armPositionRotations;
  assertTrue(afterVoltage < 0.37);

  io.setArmDutyCycle(0.5);
  update(io, inputs, 20);
  assertTrue(inputs.armAppliedVolts > 0.0);

  RoboRioSim.setVInVoltage(12.0);
  io = new IntakeIOSim();
  io.setArmTorqueCurrent(Amps.of(-15.0));
  io.updateInputs(inputs);
  assertTrue(inputs.armAppliedVolts < 0.0);
  assertTrue(inputs.armCurrentAmps > 0.0);
  assertTrue(inputs.armCurrentAmps <= 50.0 + 1e-6);
}

@Test
void combinedLoadSagsBatteryAndClampsEveryAppliedVoltage() {
  IntakeIOSim io = new IntakeIOSim();
  IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
  io.setRollerVoltage(Volts.of(12.0));
  io.setArmVoltage(Volts.of(10.0));

  io.updateInputs(inputs);

  double loadedVoltage = RoboRioSim.getVInVoltage();
  assertTrue(loadedVoltage < 12.0);
  assertTrue(Math.abs(inputs.rollerLeadAppliedVolts) <= loadedVoltage + 1e-9);
  assertTrue(Math.abs(inputs.rollerFollowerAppliedVolts) <= loadedVoltage + 1e-9);
  assertTrue(Math.abs(inputs.armAppliedVolts) <= loadedVoltage + 1e-9);
  assertTrue(inputs.rollerLeadCurrentAmps <= 80.0 + 1e-6);
  assertTrue(inputs.rollerFollowerCurrentAmps <= 80.0 + 1e-6);
  assertTrue(inputs.armCurrentAmps <= 50.0 + 1e-6);
}
```

Also assert that CANcoder position and velocity track the arm after motion, with absolute position
wrapped into `[-0.5, 0.5)` for the configured 0.5 discontinuity point.

- [ ] **Step 4: Run simulation tests and verify RED**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.intake.IntakeIOSimTest --console=plain
```

Expected: compilation fails because `IntakeIOSim` and simulation constants do not exist.

- [ ] **Step 5: Add the physical-model constants**

Add to `IntakeConstants.java`:

```java
static final DCMotor ROLLER_SIM_MOTOR = DCMotor.getKrakenX60Foc(1);
static final DCMotor ARM_SIM_MOTOR = DCMotor.getKrakenX60Foc(1);
static final double ROLLER_SIM_GEARING = 1.0;
static final double ROLLER_SIM_MOI = 0.004;
static final double ARM_GEAR_RATIO = 40.0;
static final double ARM_LENGTH_METERS = Units.inchesToMeters(13.370);
static final double ARM_MASS_KG = Units.lbsToKilograms(10.0);
static final double ARM_MIN_RADIANS = Units.rotationsToRadians(ARM_DEPLOYED_ROTATIONS);
static final double ARM_MAX_RADIANS = Units.rotationsToRadians(ARM_STOWED_ROTATIONS);
static final double ARM_START_RADIANS = Units.rotationsToRadians(ARM_STOWED_ROTATIONS);
static final double LOOP_PERIOD_SECONDS = 0.020;
```

Use `ARM_GEAR_RATIO` in the TalonFX feedback config too, so hardware and simulation cannot drift.

- [ ] **Step 6: Build the three physics models and explicit control modes**

Create two independent roller `DCMotorSim` objects from
`LinearSystemId.createDCMotorSystem(ROLLER_SIM_MOTOR, ROLLER_SIM_MOI, ROLLER_SIM_GEARING)` and one:

```java
new SingleJointedArmSim(
    ARM_SIM_MOTOR,
    ARM_GEAR_RATIO,
    SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, ARM_MASS_KG),
    ARM_LENGTH_METERS,
    ARM_MIN_RADIANS,
    ARM_MAX_RADIANS,
    true,
    ARM_START_RADIANS)
```

Track only the active request for each mechanism:

```java
private enum RollerControlMode { VOLTAGE, DUTY_CYCLE, TORQUE_CURRENT }
private enum ArmControlMode { POSITION, VOLTAGE, DUTY_CYCLE, TORQUE_CURRENT, BRAKE }
```

Initialize roller mode to zero-voltage `VOLTAGE` and arm mode to `BRAKE`, so the first telemetry
update is safe and never switches on a null enum. Store requested roller volts/duty/current/max
duty and requested arm rotations/slot/volts/duty/current. Every IO setter changes its mechanism's
active mode and clamps the incoming request at the IO boundary, matching `IntakeIOTalonFX`.

- [ ] **Step 7: Calculate controls, update physics, battery, and all telemetry**

At the start of `updateInputs`, calculate each requested voltage against the 12 V source and its
configured mechanism limit. Roller modes calculate:

```java
case VOLTAGE -> requestedRollerVoltage;
case DUTY_CYCLE -> requestedRollerDutyCycle * ROLLER_MAX_VOLTAGE.in(Volts);
case TORQUE_CURRENT -> MathUtil.clamp(
    ROLLER_SIM_MOTOR.getVoltage(
        ROLLER_SIM_MOTOR.getTorque(requestedRollerTorqueCurrentAmps),
        rollerLeadSim.getAngularVelocityRadPerSec() * ROLLER_SIM_GEARING),
    -requestedRollerMaxDuty * 12.0,
    requestedRollerMaxDuty * 12.0);
```

Apply the negative of lead voltage to the opposed follower. Arm position uses the selected slot's
approved `kP`, mechanism-rotation error, and cosine gravity feedforward:

```java
double feedbackVolts = selectedKp * (requestedArmRotations - currentArmRotations);
double gravityVolts = ARM_KG * Math.cos(armSim.getAngleRads());
yield feedbackVolts + gravityVolts;
```

Arm torque-current conversion uses `ARM_SIM_MOTOR.getVoltage(...)` with arm motor speed equal to
`armSim.getVelocityRadPerSec() * ARM_GEAR_RATIO`. BRAKE returns exactly 0 V and clears any prior
position drive.

Before battery estimation, constrain every proposed voltage to the configured stator-current limit
at its present rotor speed. Use one shared helper for all control modes and all three motors:

```java
private static double limitVoltageForStatorCurrent(
    DCMotor motor, double rotorSpeedRadPerSec, double requestedVolts, double limitAmps) {
  double requestedCurrent = motor.getCurrent(rotorSpeedRadPerSec, requestedVolts);
  if (Math.abs(requestedCurrent) <= limitAmps) return requestedVolts;
  return motor.getVoltage(
      motor.getTorque(Math.copySign(limitAmps, requestedCurrent)), rotorSpeedRadPerSec);
}
```

Estimate the three nonnegative, stator-limited draws from those proposed voltages, calculate one
loaded battery voltage, and publish it **before** sending final inputs to the physics models:

```java
double estimatedRollerLeadCurrentAmps =
    Math.min(80.0, Math.abs(ROLLER_SIM_MOTOR.getCurrent(leadRotorSpeed, leadVolts)));
double estimatedRollerFollowerCurrentAmps =
    Math.min(80.0, Math.abs(ROLLER_SIM_MOTOR.getCurrent(followerRotorSpeed, followerVolts)));
double estimatedArmCurrentAmps =
    Math.min(50.0, Math.abs(ARM_SIM_MOTOR.getCurrent(armRotorSpeed, armVolts)));
RoboRioSim.setVInVoltage(
    BatterySim.calculateDefaultBatteryLoadedVoltage(
        estimatedRollerLeadCurrentAmps,
        estimatedRollerFollowerCurrentAmps,
        estimatedArmCurrentAmps));
```

Clamp the proposed roller voltages again to both loaded bus voltage and the selected torque-duty
cap; clamp arm voltage to loaded bus voltage and ±10 V. Then call `setInputVoltage` and update all
three models by `LOOP_PERIOD_SECONDS`. This avoids the raw two-Kraken stall estimate collapsing the
battery to 0 V while still modeling the configured 80/80/50 A stator ceilings.

Populate all 22 fields on every call. Log the final post-clamp inputs with
`rollerSim.getInputVoltage()` and `armSim.getInput(0)`, not the earlier local requests. Report
absolute currents clamped to 80/80/50 A, temperatures `0.0`, and all four connections `true`.
Convert angular velocity to RPM using `armSim.getVelocityRadPerSec()` for the arm. Set CANcoder
mechanism position/velocity from `armSim.getAngleRads()` and wrap absolute position with:

```java
MathUtil.inputModulus(armPositionRotations, -0.5, 0.5)
```

- [ ] **Step 8: Run focused and intake-wide tests**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.intake.IntakeIOSimTest --console=plain
.\gradlew.bat test --tests "frc.robot.subsystems.intake.*" --console=plain
```

Expected: every Task 1 through Task 3 intake test passes.

- [ ] **Step 9: Commit full simulation**

```powershell
git add src/main/java/frc/robot/subsystems/intake/IntakeConstants.java
git add src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java
git add src/test/java/frc/robot/subsystems/intake/IntakeIOSimTest.java
git commit -m "feat(intake): add full physics simulation"
```

---

### Task 4: Runtime Mode Wiring

**Files:**

- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Create: `src/test/java/frc/robot/RobotContainerIntakeTest.java`

**Interfaces:**

- Consumes `Intake`, `IntakeIO`, `IntakeIOTalonFX`, and `IntakeIOSim`.
- Produces one retained `Intake` subsystem and testable REAL/SIM/REPLAY IO selection.
- Adds no controller bindings and does not modify Superstructure.

- [ ] **Step 1: Write failing factory-mapping tests without constructing CAN hardware**

Create `RobotContainerIntakeTest.java`:

```java
package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants.Mode;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import org.junit.jupiter.api.Test;

class RobotContainerIntakeTest {
  @Test
  void realFactoryMappingNamesHardwareIoWithoutConstructingIt() {
    assertEquals(
        IntakeIOTalonFX.class, RobotContainer.intakeIOFactory(Mode.REAL).implementationType());
  }

  @Test
  void simulationUsesFullPhysicsIo() {
    assertInstanceOf(IntakeIOSim.class, RobotContainer.createIntakeIO(Mode.SIM));
  }

  @Test
  void replayUsesNoOpIo() {
    assertInstanceOf(IntakeIO.NoOp.class, RobotContainer.createIntakeIO(Mode.REPLAY));
  }
}
```

- [ ] **Step 2: Run runtime wiring tests and verify RED**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.RobotContainerIntakeTest --console=plain
```

Expected: compilation fails because the intake factory methods do not exist.

- [ ] **Step 3: Add imports, retained subsystem, and typed factory record**

Add all four intake imports to `RobotContainer`. Add this record beside `ShooterIOFactory`:

```java
record IntakeIOFactory<T extends IntakeIO>(Class<T> implementationType, Supplier<T> constructor) {
  T create() {
    return constructor.get();
  }
}
```

Retain the subsystem with the other declarations:

```java
private final Intake intake;
private final Hopper hopper;
private final Shooter shooter;
```

After drive construction and before hopper/shooter construction, add:

```java
intake = new Intake(createIntakeIO(Constants.currentMode));
```

Do not add a getter solely for tests; factory tests provide the seam without constructing an entire
`RobotContainer` or initializing AutoBuilder/dashboard state.

- [ ] **Step 4: Add exact mode factories**

Add:

```java
static IntakeIO createIntakeIO(Constants.Mode mode) {
  return intakeIOFactory(mode).create();
}

static IntakeIOFactory<? extends IntakeIO> intakeIOFactory(Constants.Mode mode) {
  return switch (mode) {
    case REAL -> new IntakeIOFactory<>(IntakeIOTalonFX.class, IntakeIOTalonFX::new);
    case SIM -> new IntakeIOFactory<>(IntakeIOSim.class, IntakeIOSim::new);
    case REPLAY -> new IntakeIOFactory<>(IntakeIO.NoOp.class, IntakeIO.NoOp::new);
  };
}
```

- [ ] **Step 5: Run factory and intake regressions**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.RobotContainerIntakeTest --console=plain
.\gradlew.bat test --tests "frc.robot.subsystems.intake.*" --console=plain
```

Expected: runtime mapping and all intake tests pass; the REAL mapping test never constructs Phoenix
devices.

- [ ] **Step 6: Commit runtime wiring**

```powershell
git add src/main/java/frc/robot/RobotContainer.java
git add src/test/java/frc/robot/RobotContainerIntakeTest.java
git commit -m "feat(intake): wire runtime IO modes"
```

---

### Task 5: Formatting, Full Verification, and Scope Audit

**Files:**

- Verify all files listed in the File Map.
- Modify only formatter output within those approved files.

- [ ] **Step 1: Format and inspect formatter effects**

Run:

```powershell
.\gradlew.bat spotlessApply --console=plain
git status --short
git diff --stat
```

Inspect every changed path. If Spotless touched a file outside the File Map, do not stage or discard
it blindly; determine whether it was pre-existing user work and preserve it.

- [ ] **Step 2: Run the complete verification gate from a clean test result**

Run:

```powershell
.\gradlew.bat test testClasses spotlessCheck --rerun-tasks --console=plain
```

Expected: `BUILD SUCCESSFUL`, all tests pass, test classes compile, and Spotless reports no
violations. Record the exact executed/passed/skipped test counts from the XML results or Gradle
report; do not infer counts from task success.

- [ ] **Step 3: Run repository hygiene checks**

Run:

```powershell
git diff --check origin/main...HEAD
git status --short
git log --oneline --decorate -6
git diff --stat origin/main...HEAD
```

Audit `origin/main...HEAD`. It may contain only:

- the approved intake design and implementation-plan documents;
- the new intake main/test packages;
- the narrow `RobotContainer` wiring and its factory test.

Confirm there are no bindings, Superstructure changes, hopper/shooter edits, generated logs,
Gradle caches, vendordep changes, or unrelated formatting churn.

- [ ] **Step 4: Commit formatter-only changes if any exist**

If Spotless changed approved intake files after Task 4, stage only those paths and commit:

```powershell
git add src/main/java/frc/robot/subsystems/intake
git add src/test/java/frc/robot/subsystems/intake
git add src/main/java/frc/robot/RobotContainer.java
git add src/test/java/frc/robot/RobotContainerIntakeTest.java
git commit -m "style(intake): apply formatting"
```

If there is no diff, skip this commit.

- [ ] **Step 5: Request independent code review and address findings**

Read and apply `superpowers:requesting-code-review`. Give the reviewer the approved design,
implementation plan, exact base/head commits, current diff, and verification output. Require review
of state/output same-loop alignment, brake-neutral collision behavior, Phoenix refresh/frequency
coverage, all 22 telemetry fields, full control-mode simulation, and scope exclusions.

For any finding, use `superpowers:receiving-code-review`, reproduce it, add or strengthen a failing
test, make the smallest correction, and rerun the focused test plus the full Step 2 gate.

- [ ] **Step 6: Re-run final evidence gate**

After review fixes, run fresh:

```powershell
.\gradlew.bat test testClasses spotlessCheck --rerun-tasks --console=plain
git diff --check origin/main...HEAD
git status --short
git diff --stat origin/main...HEAD
```

Only report completion when all commands pass and the worktree has no uncommitted implementation
changes. Do not push, merge, or open a PR without separate user authorization.
