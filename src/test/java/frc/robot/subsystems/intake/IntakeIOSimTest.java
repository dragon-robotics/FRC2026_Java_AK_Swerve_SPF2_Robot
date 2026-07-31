package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeIOSimTest {
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
    assertTrue(Math.abs(inputs.rollerLeadAppliedVolts) <= 0.50 * RoboRioSim.getVInVoltage() + 1e-9);
    assertTrue(inputs.rollerLeadVelocityRpm > 0.0);
    assertTrue(inputs.rollerFollowerVelocityRpm < 0.0);
  }

  @Test
  void rollerDutyCycleUsesLoadedBusVoltageUnderModeledSag() {
    IntakeIOSim io = new IntakeIOSim();
    IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
    double requestedDutyCycle = 0.05;
    io.setRollerDutyCycle(requestedDutyCycle);
    RotorSpeeds previousSpeeds = captureRotorSpeeds(inputs);

    io.updateInputs(inputs);

    double loadedBusVolts = RoboRioSim.getVInVoltage();
    assertTrue(loadedBusVolts < 12.0);
    assertEquals(requestedDutyCycle, inputs.rollerLeadAppliedVolts / loadedBusVolts, 1e-9);
    assertEquals(-inputs.rollerLeadAppliedVolts, inputs.rollerFollowerAppliedVolts, 1e-9);
    assertPhysicalCurrentLimits(inputs, previousSpeeds);
    assertBatteryMatchesAppliedCycle(inputs, previousSpeeds);
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
  void armDutyCycleUsesLoadedBusVoltageAndMechanismCapUnderModeledSag() {
    IntakeIOSim io = new IntakeIOSim();
    IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
    double requestedDutyCycle = -0.04;
    io.setArmDutyCycle(requestedDutyCycle);
    RotorSpeeds previousSpeeds = captureRotorSpeeds(inputs);

    io.updateInputs(inputs);

    double loadedBusVolts = RoboRioSim.getVInVoltage();
    double uncappedDutyVolts = requestedDutyCycle * loadedBusVolts;
    assertTrue(loadedBusVolts < 12.0);
    assertTrue(Math.abs(uncappedDutyVolts) < 10.0);
    assertEquals(uncappedDutyVolts, inputs.armAppliedVolts, 1e-9);
    assertTrue(Math.abs(inputs.armAppliedVolts) <= 10.0);
    assertPhysicalCurrentLimits(inputs, previousSpeeds);
    assertBatteryMatchesAppliedCycle(inputs, previousSpeeds);
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

  @Test
  void physicalStatorLimitsConstrainAppliedInputsNotOnlyTelemetry() {
    IntakeIOSim io = new IntakeIOSim();
    IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
    io.setRollerVoltage(Volts.of(12.0));
    io.setArmVoltage(Volts.of(-10.0));

    for (int cycle = 0; cycle < 25; cycle++) {
      RotorSpeeds previousSpeeds = captureRotorSpeeds(inputs);
      io.updateInputs(inputs);
      assertPhysicalCurrentLimits(inputs, previousSpeeds);
      assertBatteryMatchesAppliedCycle(inputs, previousSpeeds);
      assertTrue(inputs.rollerLeadCurrentAmps <= 80.0 + 1e-6);
      assertTrue(inputs.rollerFollowerCurrentAmps <= 80.0 + 1e-6);
      assertTrue(inputs.armCurrentAmps <= 50.0 + 1e-6);
    }
  }

  @Test
  void activeHighSpeedReversalRemainsConsistentThroughRegenerationAndMotoring() {
    IntakeIOSim io = new IntakeIOSim();
    IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
    io.setRollerVoltage(Volts.of(12.0));
    update(io, inputs, 200);

    io.setRollerVoltage(Volts.of(-12.0));
    RotorSpeeds initialSpeeds = captureRotorSpeeds(inputs);
    io.updateInputs(inputs);
    PhysicalStatorCurrents initialStatorCurrents =
        calculatePhysicalStatorCurrents(inputs, initialSpeeds);

    assertAll(
        "initial active-reversal cycle",
        () -> assertTrue(RoboRioSim.getVInVoltage() > 12.0, "regenerative bus voltage"),
        () -> assertPhysicalCurrentLimits(inputs, initialSpeeds),
        () -> assertBatteryMatchesAppliedCycle(inputs, initialSpeeds),
        () -> assertTrue(inputs.rollerLeadAppliedVolts > 0.0, "lead regenerative voltage"),
        () -> assertTrue(inputs.rollerFollowerAppliedVolts < 0.0, "follower regenerative voltage"),
        () -> assertTrue(initialStatorCurrents.rollerLeadAmps() < 0.0, "lead braking current"),
        () ->
            assertTrue(
                initialStatorCurrents.rollerFollowerAmps() > 0.0, "follower braking current"));

    boolean sawMotoringBatteryLoad = false;
    for (int cycle = 1; cycle < 100; cycle++) {
      RotorSpeeds previousSpeeds = captureRotorSpeeds(inputs);
      io.updateInputs(inputs);
      PhysicalStatorCurrents statorCurrents =
          calculatePhysicalStatorCurrents(inputs, previousSpeeds);

      assertPhysicalCurrentLimits(inputs, previousSpeeds);
      assertBatteryMatchesAppliedCycle(inputs, previousSpeeds);
      assertEquals(-inputs.rollerLeadAppliedVolts, inputs.rollerFollowerAppliedVolts, 1e-9);
      assertEquals(-statorCurrents.rollerLeadAmps(), statorCurrents.rollerFollowerAmps(), 1e-6);
      assertEquals(-inputs.rollerLeadVelocityRpm, inputs.rollerFollowerVelocityRpm, 1e-6);
      sawMotoringBatteryLoad |= RoboRioSim.getVInVoltage() < 12.0 - 1e-9;
    }

    assertTrue(sawMotoringBatteryLoad, "reversal transitioned from regeneration to motoring");
  }

  @Test
  void cancoderTracksArmPositionAndVelocityAfterMotion() {
    IntakeIOSim io = new IntakeIOSim();
    IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
    io.setArmVoltage(Volts.of(-4.0));

    update(io, inputs, 20);

    assertEquals(inputs.armPositionRotations, inputs.armCancoderPositionRotations, 1e-9);
    assertEquals(inputs.armVelocityRpm, inputs.armCancoderVelocityRpm, 1e-9);
    assertEquals(inputs.armPositionRotations, inputs.armCancoderAbsolutePositionRotations, 1e-9);
    assertTrue(inputs.armCancoderAbsolutePositionRotations >= -0.5);
    assertTrue(inputs.armCancoderAbsolutePositionRotations < 0.5);
  }

  private static void update(IntakeIOSim io, IntakeIO.IntakeIOInputs inputs, int numberOfCycles) {
    for (int i = 0; i < numberOfCycles; i++) {
      io.updateInputs(inputs);
    }
  }

  private static RotorSpeeds captureRotorSpeeds(IntakeIO.IntakeIOInputs inputs) {
    return new RotorSpeeds(
        Units.rotationsPerMinuteToRadiansPerSecond(inputs.rollerLeadVelocityRpm)
            * IntakeConstants.ROLLER_SIM_GEARING,
        Units.rotationsPerMinuteToRadiansPerSecond(inputs.rollerFollowerVelocityRpm)
            * IntakeConstants.ROLLER_SIM_GEARING,
        Units.rotationsPerMinuteToRadiansPerSecond(inputs.armVelocityRpm)
            * IntakeConstants.ARM_GEAR_RATIO);
  }

  private static void assertPhysicalCurrentLimits(
      IntakeIO.IntakeIOInputs inputs, RotorSpeeds previousSpeeds) {
    PhysicalStatorCurrents currents = calculatePhysicalStatorCurrents(inputs, previousSpeeds);
    assertTrue(
        Math.abs(currents.rollerLeadAmps()) <= 80.0 + 1e-6, "roller lead physical stator current");
    assertTrue(
        Math.abs(currents.rollerFollowerAmps()) <= 80.0 + 1e-6,
        "roller follower physical stator current");
    assertTrue(Math.abs(currents.armAmps()) <= 50.0 + 1e-6, "arm physical stator current");
  }

  private static void assertBatteryMatchesAppliedCycle(
      IntakeIO.IntakeIOInputs inputs, RotorSpeeds previousSpeeds) {
    PhysicalStatorCurrents currents = calculatePhysicalStatorCurrents(inputs, previousSpeeds);
    assertEquals(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            currents.rollerLeadAmps() * Math.signum(inputs.rollerLeadAppliedVolts),
            currents.rollerFollowerAmps() * Math.signum(inputs.rollerFollowerAppliedVolts),
            currents.armAmps() * Math.signum(inputs.armAppliedVolts)),
        RoboRioSim.getVInVoltage(),
        1e-9);
  }

  private static PhysicalStatorCurrents calculatePhysicalStatorCurrents(
      IntakeIO.IntakeIOInputs inputs, RotorSpeeds previousSpeeds) {
    return new PhysicalStatorCurrents(
        IntakeConstants.ROLLER_SIM_MOTOR.getCurrent(
            previousSpeeds.rollerLeadRadiansPerSecond(), inputs.rollerLeadAppliedVolts),
        IntakeConstants.ROLLER_SIM_MOTOR.getCurrent(
            previousSpeeds.rollerFollowerRadiansPerSecond(), inputs.rollerFollowerAppliedVolts),
        IntakeConstants.ARM_SIM_MOTOR.getCurrent(
            previousSpeeds.armRadiansPerSecond(), inputs.armAppliedVolts));
  }

  private record RotorSpeeds(
      double rollerLeadRadiansPerSecond,
      double rollerFollowerRadiansPerSecond,
      double armRadiansPerSecond) {}

  private record PhysicalStatorCurrents(
      double rollerLeadAmps, double rollerFollowerAmps, double armAmps) {}

  private static IntakeIO.IntakeIOInputs inputsWithDistinctSentinels() {
    IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
    inputs.rollerLeadConnected = false;
    inputs.rollerLeadPositionRotations = 101.0;
    inputs.rollerLeadVelocityRpm = 102.0;
    inputs.rollerLeadAppliedVolts = 103.0;
    inputs.rollerLeadCurrentAmps = 104.0;
    inputs.rollerLeadTempCelsius = 105.0;
    inputs.rollerFollowerConnected = false;
    inputs.rollerFollowerPositionRotations = 106.0;
    inputs.rollerFollowerVelocityRpm = 107.0;
    inputs.rollerFollowerAppliedVolts = 108.0;
    inputs.rollerFollowerCurrentAmps = 109.0;
    inputs.rollerFollowerTempCelsius = 110.0;
    inputs.armConnected = false;
    inputs.armPositionRotations = 111.0;
    inputs.armVelocityRpm = 112.0;
    inputs.armAppliedVolts = 113.0;
    inputs.armCurrentAmps = 114.0;
    inputs.armTempCelsius = 115.0;
    inputs.armCancoderConnected = false;
    inputs.armCancoderPositionRotations = 116.0;
    inputs.armCancoderAbsolutePositionRotations = 117.0;
    inputs.armCancoderVelocityRpm = 118.0;
    return inputs;
  }

  private static void assertNoSentinelValuesRemain(IntakeIO.IntakeIOInputs inputs) {
    assertTrue(inputs.rollerLeadConnected);
    assertNotEquals(101.0, inputs.rollerLeadPositionRotations);
    assertNotEquals(102.0, inputs.rollerLeadVelocityRpm);
    assertNotEquals(103.0, inputs.rollerLeadAppliedVolts);
    assertNotEquals(104.0, inputs.rollerLeadCurrentAmps);
    assertNotEquals(105.0, inputs.rollerLeadTempCelsius);
    assertTrue(inputs.rollerFollowerConnected);
    assertNotEquals(106.0, inputs.rollerFollowerPositionRotations);
    assertNotEquals(107.0, inputs.rollerFollowerVelocityRpm);
    assertNotEquals(108.0, inputs.rollerFollowerAppliedVolts);
    assertNotEquals(109.0, inputs.rollerFollowerCurrentAmps);
    assertNotEquals(110.0, inputs.rollerFollowerTempCelsius);
    assertTrue(inputs.armConnected);
    assertNotEquals(111.0, inputs.armPositionRotations);
    assertNotEquals(112.0, inputs.armVelocityRpm);
    assertNotEquals(113.0, inputs.armAppliedVolts);
    assertNotEquals(114.0, inputs.armCurrentAmps);
    assertNotEquals(115.0, inputs.armTempCelsius);
    assertTrue(inputs.armCancoderConnected);
    assertNotEquals(116.0, inputs.armCancoderPositionRotations);
    assertNotEquals(117.0, inputs.armCancoderAbsolutePositionRotations);
    assertNotEquals(118.0, inputs.armCancoderVelocityRpm);
  }
}
