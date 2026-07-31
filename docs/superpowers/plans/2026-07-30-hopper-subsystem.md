# Hopper Subsystem Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an AdvantageKit-native hopper subsystem with real, simulation, and replay IO before
future Superstructure integration.

**Architecture:** `Hopper` owns desired/current state semantics and clamps its public direct-control
API. `HopperIO` separates logged telemetry and output requests from `HopperIOTalonFX` hardware and
`HopperIOSim` physics implementations. `RobotContainer` selects the correct IO by runtime mode
without adding commands or bindings.

**Tech Stack:** Java 17, WPILib 2026, AdvantageKit `@AutoLog`, CTRE Phoenix 6 v26.3.0, JUnit 5,
GradleRIO.

## Global Constraints

- Preserve TalonFX CAN IDs 17 and 18.
- Preserve +12 V index-to-shooter, -12 V index-to-intake, and 0 V stop behavior.
- Preserve 80 A stator, 40 A supply, 20 A supply-lower after 0.2 seconds, and 0.5 second open-loop
  ramp limits.
- Use voltage, duty-cycle percentage, and torque-current FOC direct controls; do not add RPM
  control.
- Configure `TorqueCurrentFOC` with 1.0 max absolute duty cycle, 1 A deadband, coast during neutral,
  and 100 Hz update frequency.
- Configure voltage and duty-cycle requests at 100 Hz.
- Configure position, velocity, applied voltage, and stator-current signals at 50 Hz.
- Configure temperature and all otherwise unspecified status signals at 4 Hz.
- Add no controller bindings, intake/shooter behavior, Superstructure, autonomous commands, fuel
  sensing, or jam detection.
- Preserve unrelated untracked design and plan files.

---

## File Map

- `src/main/java/frc/robot/subsystems/hopper/HopperConstants.java`: hardware, control, CAN-frequency,
  and simulation constants plus TalonFX configuration factories.
- `src/main/java/frc/robot/subsystems/hopper/HopperIO.java`: logged input schema and output boundary.
- `src/main/java/frc/robot/subsystems/hopper/Hopper.java`: public subsystem API and state machine.
- `src/main/java/frc/robot/subsystems/hopper/HopperIOSim.java`: two-motor physics simulation.
- `src/main/java/frc/robot/subsystems/hopper/HopperIOTalonFX.java`: real TalonFX implementation.
- `src/main/java/frc/robot/RobotContainer.java`: runtime IO selection and retained hopper instance.
- `src/test/java/frc/robot/subsystems/hopper/HopperTest.java`: state and direct-control behavior.
- `src/test/java/frc/robot/subsystems/hopper/HopperIOSimTest.java`: simulation output behavior.
- `src/test/java/frc/robot/subsystems/hopper/HopperIOTalonFXConfigTest.java`: Phoenix request and
  configuration values without constructing CAN hardware.
- `src/test/java/frc/robot/RobotContainerHopperTest.java`: simulation/replay IO-selection behavior.

---

### Task 1: Hopper Contract and State Machine

**Files:**

- Create: `src/main/java/frc/robot/subsystems/hopper/HopperConstants.java`
- Create: `src/main/java/frc/robot/subsystems/hopper/HopperIO.java`
- Create: `src/main/java/frc/robot/subsystems/hopper/Hopper.java`
- Test: `src/test/java/frc/robot/subsystems/hopper/HopperTest.java`

**Interfaces:**

- Produces: `HopperIO.updateInputs(HopperIOInputs)`, `setVoltage(Voltage)`,
  `setDutyCycle(double)`, and `setTorqueCurrent(Current)`.
- Produces: `Hopper(HopperIO)`, `setDesiredState(HopperState)`, `getDesiredState()`,
  `getCurrentState()`, `runVoltage(Voltage)`, `runPercentage(double)`,
  `runTorqueCurrentFOC(Current)`, `indexToShooter()`, `indexToIntake()`, and `stop()`.
- Produces: `HopperState.STOP`, `INDEX_TO_SHOOTER`, and `INDEX_TO_INTAKE`.

- [ ] **Step 1: Read test-quality rules**

Read:

```text
C:\Users\dougd\.codex\plugins\cache\openai-curated-remote\superpowers\6.2.0\skills\test-driven-development\writing-good-tests.md
```

Before each test, identify the production change that would make it fail.

- [ ] **Step 2: Write failing state and direct-control tests**

Create `HopperTest.java` with a recording fake:

```java
package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import org.junit.jupiter.api.Test;

class HopperTest {
  private static class RecordingIO implements HopperIO {
    int voltageCalls;
    double volts;
    double dutyCycle;
    double torqueCurrentAmps;

    @Override
    public void setVoltage(Voltage voltage) {
      voltageCalls++;
      volts = voltage.in(Volts);
    }

    @Override
    public void setDutyCycle(double output) {
      dutyCycle = output;
    }

    @Override
    public void setTorqueCurrent(Current current) {
      torqueCurrentAmps = current.in(Amps);
    }
  }

  @Test
  void firstPeriodicExplicitlyStopsHopper() {
    RecordingIO io = new RecordingIO();
    Hopper hopper = new Hopper(io);

    hopper.periodic();

    assertEquals(1, io.voltageCalls);
    assertEquals(0.0, io.volts, 1e-9);
  }

  @Test
  void stateChangesApplyOnceOnPeriodic() {
    RecordingIO io = new RecordingIO();
    Hopper hopper = new Hopper(io);
    hopper.periodic();

    hopper.setDesiredState(HopperState.INDEX_TO_SHOOTER);
    assertEquals(HopperState.STOP, hopper.getCurrentState());
    hopper.periodic();
    hopper.periodic();

    assertEquals(HopperState.INDEX_TO_SHOOTER, hopper.getCurrentState());
    assertEquals(12.0, io.volts, 1e-9);
    assertEquals(2, io.voltageCalls);
  }

  @Test
  void indexToIntakeUsesNegativeVoltage() {
    RecordingIO io = new RecordingIO();
    Hopper hopper = new Hopper(io);

    hopper.setDesiredState(HopperState.INDEX_TO_INTAKE);
    hopper.periodic();

    assertEquals(-12.0, io.volts, 1e-9);
  }

  @Test
  void directControlsClampBeforeForwarding() {
    RecordingIO io = new RecordingIO();
    Hopper hopper = new Hopper(io);

    hopper.runVoltage(Volts.of(18.0));
    hopper.runPercentage(-2.0);
    hopper.runTorqueCurrentFOC(Amps.of(120.0));

    assertEquals(12.0, io.volts, 1e-9);
    assertEquals(-1.0, io.dutyCycle, 1e-9);
    assertEquals(80.0, io.torqueCurrentAmps, 1e-9);
  }

  @Test
  void nullDesiredStateIsRejected() {
    Hopper hopper = new Hopper(new RecordingIO());
    assertThrows(NullPointerException.class, () -> hopper.setDesiredState(null));
  }
}
```

- [ ] **Step 3: Run tests and verify RED**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.hopper.HopperTest
```

Expected: compilation fails because `Hopper`, `HopperIO`, and `HopperState` do not exist.

- [ ] **Step 4: Add constants and IO contract**

Create `HopperConstants.java` with typed values:

```java
package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

final class HopperConstants {
  static final int LEAD_MOTOR_ID = 17;
  static final int FOLLOWER_MOTOR_ID = 18;
  static final Current STATOR_CURRENT_LIMIT = Amps.of(80.0);
  static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40.0);
  static final Current SUPPLY_CURRENT_LOWER_LIMIT = Amps.of(20.0);
  static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.2);
  static final Voltage MAX_VOLTAGE = Volts.of(12.0);
  static final Voltage INDEX_TO_SHOOTER_VOLTAGE = Volts.of(12.0);
  static final Voltage INDEX_TO_INTAKE_VOLTAGE = Volts.of(-12.0);
  static final Voltage STOP_VOLTAGE = Volts.zero();
  static final Time OPEN_LOOP_RAMP = Seconds.of(0.5);
  static final double MAX_ABS_DUTY_CYCLE = 1.0;
  static final Current TORQUE_DEADBAND = Amps.of(1.0);
  static final double CONTROL_UPDATE_HZ = 100.0;
  static final double MECHANISM_STATUS_HZ = 50.0;
  static final double SLOW_STATUS_HZ = 4.0;
  static final DCMotor SIM_MOTOR = DCMotor.getKrakenX60Foc(1);
  static final double SIM_GEARING = 1.0;
  static final double SIM_MOI_KG_METERS_SQUARED = 0.004;

  private HopperConstants() {}
}
```

Create `HopperIO.java`:

```java
package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  class HopperIOInputs {
    public boolean leadConnected;
    public double leadPositionRotations;
    public double leadVelocityRpm;
    public double leadAppliedVolts;
    public double leadCurrentAmps;
    public double leadTempCelsius;
    public boolean followerConnected;
    public double followerPositionRotations;
    public double followerVelocityRpm;
    public double followerAppliedVolts;
    public double followerCurrentAmps;
    public double followerTempCelsius;
  }

  default void updateInputs(HopperIOInputs inputs) {}

  default void setVoltage(Voltage voltage) {}

  default void setDutyCycle(double output) {}

  default void setTorqueCurrent(Current current) {}
}
```

- [ ] **Step 5: Implement minimal state machine**

Create `Hopper.java`. Clamp in this public boundary so every IO implementation receives safe values:

```java
package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hopper.HopperConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  public enum HopperState {
    STOP,
    INDEX_TO_SHOOTER,
    INDEX_TO_INTAKE
  }

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  private HopperState desiredState = HopperState.STOP;
  private HopperState currentState = HopperState.STOP;
  private HopperState lastCommandedState;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  public HopperState getDesiredState() {
    return desiredState;
  }

  public HopperState getCurrentState() {
    return currentState;
  }

  public void setDesiredState(HopperState state) {
    desiredState = Objects.requireNonNull(state);
  }

  public void runVoltage(Voltage voltage) {
    io.setVoltage(
        Volts.of(MathUtil.clamp(voltage.in(Volts), -MAX_VOLTAGE.in(Volts), MAX_VOLTAGE.in(Volts))));
  }

  public void runPercentage(double percentage) {
    io.setDutyCycle(MathUtil.clamp(percentage, -1.0, 1.0));
  }

  public void runTorqueCurrentFOC(Current current) {
    io.setTorqueCurrent(
        Amps.of(
            MathUtil.clamp(
                current.in(Amps),
                -STATOR_CURRENT_LIMIT.in(Amps),
                STATOR_CURRENT_LIMIT.in(Amps))));
  }

  public void indexToShooter() {
    runVoltage(INDEX_TO_SHOOTER_VOLTAGE);
  }

  public void indexToIntake() {
    runVoltage(INDEX_TO_INTAKE_VOLTAGE);
  }

  public void stop() {
    runVoltage(STOP_VOLTAGE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    if (desiredState != lastCommandedState) {
      switch (desiredState) {
        case STOP -> stop();
        case INDEX_TO_SHOOTER -> indexToShooter();
        case INDEX_TO_INTAKE -> indexToIntake();
      }
      currentState = desiredState;
      lastCommandedState = desiredState;
    }
    Logger.recordOutput("Hopper/CurrentState", currentState.name());
    Logger.recordOutput("Hopper/DesiredState", desiredState.name());
  }
}
```

- [ ] **Step 6: Run tests and verify GREEN**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.hopper.HopperTest
```

Expected: all five tests pass.

- [ ] **Step 7: Commit core subsystem**

```powershell
git add src/main/java/frc/robot/subsystems/hopper/HopperConstants.java
git add src/main/java/frc/robot/subsystems/hopper/HopperIO.java
git add src/main/java/frc/robot/subsystems/hopper/Hopper.java
git add src/test/java/frc/robot/subsystems/hopper/HopperTest.java
git commit -m "feat(hopper): add state and IO contract"
```

---

### Task 2: Hopper Physics Simulation

**Files:**

- Create: `src/main/java/frc/robot/subsystems/hopper/HopperIOSim.java`
- Test: `src/test/java/frc/robot/subsystems/hopper/HopperIOSimTest.java`

**Interfaces:**

- Consumes: `HopperIO` and `HopperConstants`.
- Produces: `HopperIOSim`, supporting voltage, percentage, and torque-current control.

- [ ] **Step 1: Write failing simulation tests**

Create `HopperIOSimTest.java`:

```java
package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class HopperIOSimTest {
  @Test
  void voltageIsClampedAndMovesBothMotors() {
    HopperIOSim io = new HopperIOSim();
    HopperIO.HopperIOInputs inputs = new HopperIO.HopperIOInputs();

    io.setVoltage(Volts.of(20.0));
    for (int i = 0; i < 10; i++) {
      io.updateInputs(inputs);
    }

    assertEquals(12.0, inputs.leadAppliedVolts, 1e-9);
    assertEquals(12.0, inputs.followerAppliedVolts, 1e-9);
    assertTrue(inputs.leadVelocityRpm > 0.0);
    assertTrue(inputs.followerVelocityRpm > 0.0);
  }

  @Test
  void torqueCurrentIsClampedAndMovesMotor() {
    HopperIOSim io = new HopperIOSim();
    HopperIO.HopperIOInputs inputs = new HopperIO.HopperIOInputs();

    io.setTorqueCurrent(Amps.of(120.0));
    io.updateInputs(inputs);

    assertTrue(inputs.leadCurrentAmps <= 80.0 + 1e-6);
    assertTrue(inputs.leadAppliedVolts > 0.0);
  }
}
```

- [ ] **Step 2: Run simulation tests and verify RED**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.hopper.HopperIOSimTest
```

Expected: compilation fails because `HopperIOSim` does not exist.

- [ ] **Step 3: Implement two-motor simulation**

Create `HopperIOSim.java` using `DCMotorSim` and a retained control mode. For torque current,
convert requested current to motor torque and then to the voltage needed at current simulated speed:

```java
package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hopper.HopperConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HopperIOSim implements HopperIO {
  private enum ControlMode {
    VOLTAGE,
    DUTY_CYCLE,
    TORQUE_CURRENT
  }

  private final DCMotorSim leadSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              SIM_MOTOR, SIM_MOI_KG_METERS_SQUARED, SIM_GEARING),
          SIM_MOTOR);
  private final DCMotorSim followerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              SIM_MOTOR, SIM_MOI_KG_METERS_SQUARED, SIM_GEARING),
          SIM_MOTOR);

  private ControlMode controlMode = ControlMode.VOLTAGE;
  private double requestedVoltage;
  private double requestedDutyCycle;
  private double requestedTorqueCurrentAmps;

  @Override
  public void setVoltage(Voltage voltage) {
    controlMode = ControlMode.VOLTAGE;
    requestedVoltage = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
  }

  @Override
  public void setDutyCycle(double output) {
    controlMode = ControlMode.DUTY_CYCLE;
    requestedDutyCycle = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setTorqueCurrent(Current current) {
    controlMode = ControlMode.TORQUE_CURRENT;
    requestedTorqueCurrentAmps = MathUtil.clamp(current.in(Amps), -80.0, 80.0);
  }

  private double calculateVoltage(DCMotorSim sim) {
    return switch (controlMode) {
      case VOLTAGE -> requestedVoltage;
      case DUTY_CYCLE -> requestedDutyCycle * 12.0;
      case TORQUE_CURRENT ->
          MathUtil.clamp(
              SIM_MOTOR.getVoltage(
                  SIM_MOTOR.getTorque(requestedTorqueCurrentAmps),
                  sim.getAngularVelocityRadPerSec() * SIM_GEARING),
              -12.0,
              12.0);
    };
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    double leadVolts = calculateVoltage(leadSim);
    double followerVolts = calculateVoltage(followerSim);
    leadSim.setInputVoltage(leadVolts);
    followerSim.setInputVoltage(followerVolts);
    leadSim.update(0.02);
    followerSim.update(0.02);

    inputs.leadConnected = true;
    inputs.leadPositionRotations = leadSim.getAngularPositionRotations();
    inputs.leadVelocityRpm = leadSim.getAngularVelocityRPM();
    inputs.leadAppliedVolts = leadVolts;
    inputs.leadCurrentAmps = Math.abs(leadSim.getCurrentDrawAmps());
    inputs.followerConnected = true;
    inputs.followerPositionRotations = followerSim.getAngularPositionRotations();
    inputs.followerVelocityRpm = followerSim.getAngularVelocityRPM();
    inputs.followerAppliedVolts = followerVolts;
    inputs.followerCurrentAmps = Math.abs(followerSim.getCurrentDrawAmps());
  }
}
```

- [ ] **Step 4: Run focused tests and verify GREEN**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.hopper.HopperIOSimTest
```

Expected: both tests pass. If numerical tolerance exposes motor-model transients, adjust assertions
to observable direction and configured bounds, not internal state.

- [ ] **Step 5: Commit simulation**

```powershell
git add src/main/java/frc/robot/subsystems/hopper/HopperIOSim.java
git add src/test/java/frc/robot/subsystems/hopper/HopperIOSimTest.java
git commit -m "feat(hopper): add physics simulation"
```

---

### Task 3: TalonFX Hardware IO and CAN Optimization

**Files:**

- Modify: `src/main/java/frc/robot/subsystems/hopper/HopperConstants.java`
- Create: `src/main/java/frc/robot/subsystems/hopper/HopperIOTalonFX.java`
- Test: `src/test/java/frc/robot/subsystems/hopper/HopperIOTalonFXConfigTest.java`

**Interfaces:**

- Consumes: `HopperIO`, `HopperConstants`, and `PhoenixUtil.tryUntilOk`.
- Produces: real lead/follower TalonFX IO with testable request/configuration factories.

- [ ] **Step 1: Write failing request/configuration tests**

Create `HopperIOTalonFXConfigTest.java`. Test factory outputs without constructing `TalonFX`:

```java
package frc.robot.subsystems.hopper;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import org.junit.jupiter.api.Test;

class HopperIOTalonFXConfigTest {
  @Test
  void torqueRequestUsesLockedSettings() {
    TorqueCurrentFOC request = HopperIOTalonFX.createTorqueCurrentRequest();
    assertEquals(1.0, request.MaxAbsDutyCycle, 1e-9);
    assertEquals(1.0, request.Deadband, 1e-9);
    assertTrue(request.OverrideCoastDurNeutral);
    assertEquals(100.0, request.UpdateFreqHz, 1e-9);
  }

  @Test
  void openLoopRequestsUpdateAt100Hz() {
    VoltageOut voltage = HopperIOTalonFX.createVoltageRequest();
    DutyCycleOut dutyCycle = HopperIOTalonFX.createDutyCycleRequest();
    assertEquals(100.0, voltage.UpdateFreqHz, 1e-9);
    assertEquals(100.0, dutyCycle.UpdateFreqHz, 1e-9);
  }

  @Test
  void motorConfigsPreserveReferenceLimitsAndInversions() {
    var lead = HopperConstants.createLeadConfig();
    var follower = HopperConstants.createFollowerConfig();
    assertEquals(80.0, lead.CurrentLimits.StatorCurrentLimit, 1e-9);
    assertEquals(40.0, lead.CurrentLimits.SupplyCurrentLimit, 1e-9);
    assertEquals(20.0, lead.CurrentLimits.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.2, lead.CurrentLimits.SupplyCurrentLowerTime, 1e-9);
    assertEquals(0.5, lead.OpenLoopRamps.VoltageOpenLoopRampPeriod, 1e-9);
    assertEquals(InvertedValue.CounterClockwise_Positive, lead.MotorOutput.Inverted);
    assertEquals(InvertedValue.Clockwise_Positive, follower.MotorOutput.Inverted);
  }
}
```

- [ ] **Step 2: Run hardware configuration tests and verify RED**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.hopper.HopperIOTalonFXConfigTest
```

Expected: compilation fails because hardware IO and configuration factories do not exist.

- [ ] **Step 3: Add TalonFX configuration factories**

Add `createLeadConfig()`, `createFollowerConfig()`, and a shared private builder to
`HopperConstants`. Use:

```java
new TalonFXConfiguration()
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_TIME))
    .withVoltage(
        new VoltageConfigs()
            .withPeakForwardVoltage(MAX_VOLTAGE)
            .withPeakReverseVoltage(MAX_VOLTAGE.unaryMinus()))
    .withOpenLoopRamps(
        new OpenLoopRampsConfigs()
            .withDutyCycleOpenLoopRampPeriod(OPEN_LOOP_RAMP)
            .withTorqueOpenLoopRampPeriod(OPEN_LOOP_RAMP)
            .withVoltageOpenLoopRampPeriod(OPEN_LOOP_RAMP))
    .withMotorOutput(
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(inversion));
```

- [ ] **Step 4: Implement real hardware IO**

Create `HopperIOTalonFX.java` with:

```java
static VoltageOut createVoltageRequest() {
  return new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(CONTROL_UPDATE_HZ);
}

static DutyCycleOut createDutyCycleRequest() {
  return new DutyCycleOut(0.0).withEnableFOC(true).withUpdateFreqHz(CONTROL_UPDATE_HZ);
}

static TorqueCurrentFOC createTorqueCurrentRequest() {
  return new TorqueCurrentFOC(0.0)
      .withMaxAbsDutyCycle(MAX_ABS_DUTY_CYCLE)
      .withDeadband(TORQUE_DEADBAND)
      .withOverrideCoastDurNeutral(true)
      .withUpdateFreqHz(CONTROL_UPDATE_HZ);
}
```

Constructor sequence:

1. Create lead ID 17 and follower ID 18 TalonFX objects.
2. Apply lead and follower configs with `tryUntilOk(5, ...)`.
3. Clear sticky faults with bounded retries.
4. Create position, velocity, voltage, stator-current, and temperature signals for both motors.
5. Set position, velocity, voltage, and stator-current signals to 50 Hz.
6. Set temperature signals to 4 Hz.
7. Call `ParentDevice.optimizeBusUtilizationForAll(4.0, leadMotor, followerMotor)`.
8. Send `new Follower(LEAD_MOTOR_ID, false).withUpdateFreqHz(CONTROL_UPDATE_HZ)` to follower.

Declare and initialize the hardware and signal fields with:

```java
private final TalonFX leadMotor = new TalonFX(LEAD_MOTOR_ID);
private final TalonFX followerMotor = new TalonFX(FOLLOWER_MOTOR_ID);
private final VoltageOut voltageRequest = createVoltageRequest();
private final DutyCycleOut dutyCycleRequest = createDutyCycleRequest();
private final TorqueCurrentFOC torqueCurrentRequest = createTorqueCurrentRequest();

private final StatusSignal<Angle> leadPosition = leadMotor.getPosition();
private final StatusSignal<AngularVelocity> leadVelocity = leadMotor.getVelocity();
private final StatusSignal<Voltage> leadVoltage = leadMotor.getMotorVoltage();
private final StatusSignal<Current> leadCurrent = leadMotor.getStatorCurrent();
private final StatusSignal<Temperature> leadTemperature = leadMotor.getDeviceTemp();
private final StatusSignal<Angle> followerPosition = followerMotor.getPosition();
private final StatusSignal<AngularVelocity> followerVelocity = followerMotor.getVelocity();
private final StatusSignal<Voltage> followerVoltage = followerMotor.getMotorVoltage();
private final StatusSignal<Current> followerCurrent = followerMotor.getStatorCurrent();
private final StatusSignal<Temperature> followerTemperature = followerMotor.getDeviceTemp();
private final Debouncer leadConnectedDebouncer =
    new Debouncer(0.5, Debouncer.DebounceType.kFalling);
private final Debouncer followerConnectedDebouncer =
    new Debouncer(0.5, Debouncer.DebounceType.kFalling);

public HopperIOTalonFX() {
  tryUntilOk(5, () -> leadMotor.getConfigurator().apply(createLeadConfig(), 0.25));
  tryUntilOk(5, () -> followerMotor.getConfigurator().apply(createFollowerConfig(), 0.25));
  tryUntilOk(5, () -> leadMotor.clearStickyFaults(0.25));
  tryUntilOk(5, () -> followerMotor.clearStickyFaults(0.25));

  BaseStatusSignal.setUpdateFrequencyForAll(
      MECHANISM_STATUS_HZ,
      leadPosition,
      leadVelocity,
      leadVoltage,
      leadCurrent,
      followerPosition,
      followerVelocity,
      followerVoltage,
      followerCurrent);
  BaseStatusSignal.setUpdateFrequencyForAll(
      SLOW_STATUS_HZ, leadTemperature, followerTemperature);
  ParentDevice.optimizeBusUtilizationForAll(SLOW_STATUS_HZ, leadMotor, followerMotor);

  tryUntilOk(
      5,
      () ->
          followerMotor.setControl(
              new Follower(LEAD_MOTOR_ID, false).withUpdateFreqHz(CONTROL_UPDATE_HZ)));
}
```

Implement `updateInputs` with cached 4 Hz temperature reads. Do not include temperature in the
50 Hz `refreshAll` groups, because waiting for a 4 Hz signal in every 20 ms loop could block:

```java
@Override
public void updateInputs(HopperIOInputs inputs) {
  var leadStatus =
      BaseStatusSignal.refreshAll(leadPosition, leadVelocity, leadVoltage, leadCurrent);
  var followerStatus =
      BaseStatusSignal.refreshAll(
          followerPosition, followerVelocity, followerVoltage, followerCurrent);

  inputs.leadConnected = leadConnectedDebouncer.calculate(leadStatus.isOK());
  inputs.leadPositionRotations = leadPosition.getValueAsDouble();
  inputs.leadVelocityRpm = leadVelocity.getValueAsDouble() * 60.0;
  inputs.leadAppliedVolts = leadVoltage.getValueAsDouble();
  inputs.leadCurrentAmps = leadCurrent.getValueAsDouble();
  inputs.leadTempCelsius = leadTemperature.getValueAsDouble();

  inputs.followerConnected = followerConnectedDebouncer.calculate(followerStatus.isOK());
  inputs.followerPositionRotations = followerPosition.getValueAsDouble();
  inputs.followerVelocityRpm = followerVelocity.getValueAsDouble() * 60.0;
  inputs.followerAppliedVolts = followerVoltage.getValueAsDouble();
  inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();
  inputs.followerTempCelsius = followerTemperature.getValueAsDouble();
}
```

Output methods command only the lead motor:

```java
@Override
public void setVoltage(Voltage voltage) {
  leadMotor.setControl(
      voltageRequest.withOutput(
          MathUtil.clamp(voltage.in(Volts), -MAX_VOLTAGE.in(Volts), MAX_VOLTAGE.in(Volts))));
}

@Override
public void setDutyCycle(double output) {
  leadMotor.setControl(dutyCycleRequest.withOutput(MathUtil.clamp(output, -1.0, 1.0)));
}

@Override
public void setTorqueCurrent(Current current) {
  leadMotor.setControl(
      torqueCurrentRequest.withOutput(
          MathUtil.clamp(
              current.in(Amps),
              -STATOR_CURRENT_LIMIT.in(Amps),
              STATOR_CURRENT_LIMIT.in(Amps))));
}
```

- [ ] **Step 5: Run hardware and regression tests**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.hopper.HopperIOTalonFXConfigTest
.\gradlew.bat test --tests "frc.robot.subsystems.hopper.*"
```

Expected: all hardware configuration and hopper tests pass without constructing CAN devices.

- [ ] **Step 6: Commit hardware IO**

```powershell
git add src/main/java/frc/robot/subsystems/hopper/HopperConstants.java
git add src/main/java/frc/robot/subsystems/hopper/HopperIOTalonFX.java
git add src/test/java/frc/robot/subsystems/hopper/HopperIOTalonFXConfigTest.java
git commit -m "feat(hopper): add TalonFX hardware IO"
```

---

### Task 4: Runtime Wiring

**Files:**

- Modify: `src/main/java/frc/robot/RobotContainer.java:33-113`
- Test: `src/test/java/frc/robot/RobotContainerHopperTest.java`

**Interfaces:**

- Consumes: `Hopper`, `HopperIO`, `HopperIOSim`, `HopperIOTalonFX`, and `Constants.Mode`.
- Produces: retained `private final Hopper hopper` and package-private
  `static HopperIO createHopperIO(Mode mode)`.

- [ ] **Step 1: Write failing mode-selection test**

Create:

```java
package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants.Mode;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import org.junit.jupiter.api.Test;

class RobotContainerHopperTest {
  @Test
  void simulationUsesPhysicsIO() {
    assertInstanceOf(HopperIOSim.class, RobotContainer.createHopperIO(Mode.SIM));
  }

  @Test
  void replayUsesNoOpIO() {
    HopperIO io = RobotContainer.createHopperIO(Mode.REPLAY);
    assertNotNull(io);
    assertFalse(io instanceof HopperIOSim);
  }
}
```

- [ ] **Step 2: Run mode-selection test and verify RED**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.RobotContainerHopperTest
```

Expected: compilation fails because `createHopperIO` does not exist.

- [ ] **Step 3: Wire hopper into `RobotContainer`**

Add imports and field:

```java
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOTalonFX;

private final Hopper hopper;
```

After the drive mode switch:

```java
hopper = new Hopper(createHopperIO(Constants.currentMode));
```

Add package-private factory:

```java
static HopperIO createHopperIO(Constants.Mode mode) {
  return switch (mode) {
    case REAL -> new HopperIOTalonFX();
    case SIM -> new HopperIOSim();
    case REPLAY -> new HopperIO() {};
  };
}
```

Do not add a default command, binding, named command, or Superstructure.

- [ ] **Step 4: Run focused and full tests**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.RobotContainerHopperTest
.\gradlew.bat test
```

Expected: all tests pass.

- [ ] **Step 5: Commit runtime wiring**

```powershell
git add src/main/java/frc/robot/RobotContainer.java
git add src/test/java/frc/robot/RobotContainerHopperTest.java
git commit -m "feat(hopper): wire runtime IO modes"
```

---

### Task 5: Formatting and Final Verification

**Files:**

- Modify only hopper files if Spotless changes formatting.

**Interfaces:**

- Consumes all previous tasks.
- Produces a clean, verified feature branch without touching unrelated untracked docs.

- [ ] **Step 1: Apply formatting**

Run:

```powershell
.\gradlew.bat spotlessApply
```

Then inspect:

```powershell
git status --short
git diff --stat
```

Stage only formatting changes in hopper files or `RobotContainer.java`. Do not stage unrelated
existing docs.

- [ ] **Step 2: Run final verification**

Run:

```powershell
.\gradlew.bat test
.\gradlew.bat testClasses
git diff --check
```

Expected:

- All hopper and project tests pass.
- `testClasses` succeeds.
- `git diff --check` prints nothing.

- [ ] **Step 3: Inspect final scope**

Run:

```powershell
git status --short
git log --oneline --decorate -8
```

Confirm only intended hopper implementation, tests, `RobotContainer` wiring, and this plan belong to
the feature. Existing untracked July 23/24 specs and plan files remain untouched.

- [ ] **Step 4: Commit formatter-only changes if present**

If Spotless modified already-committed hopper files:

```powershell
git add src/main/java/frc/robot/subsystems/hopper
git add src/test/java/frc/robot/subsystems/hopper
git add src/main/java/frc/robot/RobotContainer.java
git add src/test/java/frc/robot/RobotContainerHopperTest.java
git commit -m "style(hopper): apply formatting"
```

Skip this commit when no formatting diff exists.
