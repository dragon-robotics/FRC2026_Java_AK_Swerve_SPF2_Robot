package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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
  void appliedVoltageTelemetryReflectsReducedBatteryVoltage() {
    ShooterIOSim io = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    double originalBatteryVoltage = RoboRioSim.getVInVoltage();

    try {
      RoboRioSim.setVInVoltage(6.0);
      io.setFlywheelVoltage(Volts.of(12.0));
      io.setKickerVoltage(Volts.of(12.0));
      io.setHoodVoltage(Volts.of(10.0));

      io.updateInputs(inputs);

      assertEquals(6.0, inputs.flywheelLeadAppliedVolts, 1e-9);
      assertEquals(-6.0, inputs.flywheelFollowerAppliedVolts, 1e-9);
      assertEquals(6.0, inputs.kickerAppliedVolts, 1e-9);
      assertEquals(6.0, inputs.hoodAppliedVolts, 1e-9);
    } finally {
      RoboRioSim.setVInVoltage(originalBatteryVoltage);
    }
  }

  @Test
  void flywheelDutyCycleMovesOpposedMotors() {
    ShooterIOSim io = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    io.setFlywheelDutyCycle(0.5);

    io.updateInputs(inputs);

    assertEquals(6.0, inputs.flywheelLeadAppliedVolts, 1e-9);
    assertEquals(-6.0, inputs.flywheelFollowerAppliedVolts, 1e-9);
    assertTrue(inputs.flywheelLeadVelocityRpm > 0.0);
    assertTrue(inputs.flywheelFollowerVelocityRpm < 0.0);
  }

  @Test
  void flywheelTorqueCurrentMovesOpposedMotors() {
    ShooterIOSim io = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    io.setFlywheelTorqueCurrent(Amps.of(20.0));

    io.updateInputs(inputs);

    assertTrue(inputs.flywheelLeadAppliedVolts > 0.0);
    assertTrue(inputs.flywheelFollowerAppliedVolts < 0.0);
    assertTrue(inputs.flywheelLeadCurrentAmps > 0.0);
    assertTrue(inputs.flywheelLeadVelocityRpm > 0.0);
    assertTrue(inputs.flywheelFollowerVelocityRpm < 0.0);
  }

  @Test
  void kickerVoltageMovesKicker() {
    ShooterIOSim io = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    io.setKickerVoltage(Volts.of(-4.0));

    io.updateInputs(inputs);

    assertEquals(-4.0, inputs.kickerAppliedVolts, 1e-9);
    assertTrue(inputs.kickerVelocityRpm < 0.0);
  }

  @Test
  void kickerTorqueCurrentMovesKicker() {
    ShooterIOSim io = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    io.setKickerTorqueCurrent(Amps.of(-20.0));

    io.updateInputs(inputs);

    assertTrue(inputs.kickerAppliedVolts < 0.0);
    assertTrue(inputs.kickerCurrentAmps > 0.0);
    assertTrue(inputs.kickerVelocityRpm < 0.0);
  }

  @Test
  void hoodVoltageMovesHood() {
    ShooterIOSim io = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    io.setHoodVoltage(Volts.of(-4.0));

    io.updateInputs(inputs);

    assertEquals(-4.0, inputs.hoodAppliedVolts, 1e-9);
    assertTrue(inputs.hoodVelocityRpm < 0.0);
  }

  @Test
  void hoodDutyCycleMovesHood() {
    ShooterIOSim io = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    io.setHoodDutyCycle(0.5);

    io.updateInputs(inputs);

    assertEquals(5.0, inputs.hoodAppliedVolts, 1e-9);
    assertTrue(inputs.hoodVelocityRpm > 0.0);
  }

  @Test
  void resetHoodPositionUsesRequestedNonzeroPosition() {
    ShooterIOSim io = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    io.resetHoodPosition(0.625);

    io.updateInputs(inputs);

    assertEquals(0.625, inputs.hoodPositionRotations, 1e-9);
    assertEquals(0.0, inputs.hoodVelocityRpm, 1e-9);
  }

  @Test
  void updateInputsOverwritesEveryTelemetryField() {
    ShooterIOSim io = new ShooterIOSim();
    ShooterIO.ShooterIOInputs inputs = inputsWithSentinelValues();

    io.updateInputs(inputs);

    assertTrue(inputs.flywheelLeadConnected);
    assertNotEquals(101.0, inputs.flywheelLeadPositionRotations);
    assertNotEquals(102.0, inputs.flywheelLeadVelocityRpm);
    assertNotEquals(103.0, inputs.flywheelLeadAppliedVolts);
    assertNotEquals(104.0, inputs.flywheelLeadCurrentAmps);
    assertNotEquals(105.0, inputs.flywheelLeadTempCelsius);
    assertTrue(inputs.flywheelFollowerConnected);
    assertNotEquals(106.0, inputs.flywheelFollowerPositionRotations);
    assertNotEquals(107.0, inputs.flywheelFollowerVelocityRpm);
    assertNotEquals(108.0, inputs.flywheelFollowerAppliedVolts);
    assertNotEquals(109.0, inputs.flywheelFollowerCurrentAmps);
    assertNotEquals(110.0, inputs.flywheelFollowerTempCelsius);
    assertTrue(inputs.kickerConnected);
    assertNotEquals(111.0, inputs.kickerPositionRotations);
    assertNotEquals(112.0, inputs.kickerVelocityRpm);
    assertNotEquals(113.0, inputs.kickerAppliedVolts);
    assertNotEquals(114.0, inputs.kickerCurrentAmps);
    assertNotEquals(115.0, inputs.kickerTempCelsius);
    assertTrue(inputs.hoodConnected);
    assertNotEquals(116.0, inputs.hoodPositionRotations);
    assertNotEquals(117.0, inputs.hoodVelocityRpm);
    assertNotEquals(118.0, inputs.hoodAppliedVolts);
    assertNotEquals(119.0, inputs.hoodCurrentAmps);
    assertNotEquals(120.0, inputs.hoodTempCelsius);
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

  private static ShooterIO.ShooterIOInputs inputsWithSentinelValues() {
    ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
    inputs.flywheelLeadConnected = false;
    inputs.flywheelLeadPositionRotations = 101.0;
    inputs.flywheelLeadVelocityRpm = 102.0;
    inputs.flywheelLeadAppliedVolts = 103.0;
    inputs.flywheelLeadCurrentAmps = 104.0;
    inputs.flywheelLeadTempCelsius = 105.0;
    inputs.flywheelFollowerConnected = false;
    inputs.flywheelFollowerPositionRotations = 106.0;
    inputs.flywheelFollowerVelocityRpm = 107.0;
    inputs.flywheelFollowerAppliedVolts = 108.0;
    inputs.flywheelFollowerCurrentAmps = 109.0;
    inputs.flywheelFollowerTempCelsius = 110.0;
    inputs.kickerConnected = false;
    inputs.kickerPositionRotations = 111.0;
    inputs.kickerVelocityRpm = 112.0;
    inputs.kickerAppliedVolts = 113.0;
    inputs.kickerCurrentAmps = 114.0;
    inputs.kickerTempCelsius = 115.0;
    inputs.hoodConnected = false;
    inputs.hoodPositionRotations = 116.0;
    inputs.hoodVelocityRpm = 117.0;
    inputs.hoodAppliedVolts = 118.0;
    inputs.hoodCurrentAmps = 119.0;
    inputs.hoodTempCelsius = 120.0;
    return inputs;
  }
}
