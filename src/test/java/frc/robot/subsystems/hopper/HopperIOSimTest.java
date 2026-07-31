package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class HopperIOSimTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

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

  @Test
  void negativeDutyCycleIsClampedAndMovesBothMotorsBackward() {
    HopperIOSim io = new HopperIOSim();
    HopperIO.HopperIOInputs inputs = new HopperIO.HopperIOInputs();

    io.setDutyCycle(-2.0);
    for (int i = 0; i < 10; i++) {
      io.updateInputs(inputs);
    }

    assertEquals(-12.0, inputs.leadAppliedVolts, 1e-9);
    assertEquals(-12.0, inputs.followerAppliedVolts, 1e-9);
    assertTrue(inputs.leadVelocityRpm < 0.0);
    assertTrue(inputs.followerVelocityRpm < 0.0);
  }
}
