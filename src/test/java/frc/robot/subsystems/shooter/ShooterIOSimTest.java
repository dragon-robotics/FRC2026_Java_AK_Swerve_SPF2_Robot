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
