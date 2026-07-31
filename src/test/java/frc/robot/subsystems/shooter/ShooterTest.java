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
        IllegalArgumentException.class, () -> shooter.setDesiredState(ShooterState.TRANSITION));
  }

  @Test
  void rejectedTransitionRequestLeavesDesiredStateUsable() {
    RecordingIO io = new RecordingIO();
    Shooter shooter = new Shooter(io);

    assertThrows(
        IllegalArgumentException.class, () -> shooter.setDesiredState(ShooterState.TRANSITION));

    assertEquals(ShooterState.STOP, shooter.getDesiredState());
    assertDoesNotThrow(shooter::periodic);
    assertEquals(ShooterState.STOP, shooter.getCurrentState());
  }

  @Test
  void changedShootSetpointReturnsToTransitionUntilNewTargetIsReady() {
    RecordingIO io = new RecordingIO();
    Shooter shooter = new Shooter(io);
    shooter.periodic();
    shooter.setDesiredState(ShooterState.SHOOT);
    io.measuredFlywheelRpm = 2500.0;
    shooter.periodic();
    shooter.periodic();
    assertEquals(12.0, io.kickerVolts, 1e-9);

    shooter.setSetpoint(3000.0, 1.0);
    shooter.periodic();

    assertEquals(ShooterState.TRANSITION, shooter.getCurrentState());
    assertEquals(6.0, io.kickerVolts, 1e-9);
  }
}
