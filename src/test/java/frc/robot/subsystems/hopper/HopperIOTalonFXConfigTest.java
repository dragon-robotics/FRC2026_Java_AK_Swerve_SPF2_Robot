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
