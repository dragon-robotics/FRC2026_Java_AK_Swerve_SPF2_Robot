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
