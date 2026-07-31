package frc.robot.subsystems.hopper;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    assertTrue(voltage.EnableFOC);
    assertTrue(dutyCycle.EnableFOC);
    assertEquals(100.0, voltage.UpdateFreqHz, 1e-9);
    assertEquals(100.0, dutyCycle.UpdateFreqHz, 1e-9);
  }

  @Test
  void motorConfigsPreserveSafetyAndOutputContract() {
    var lead = HopperConstants.createLeadConfig();
    var follower = HopperConstants.createFollowerConfig();

    assertSafetyAndOutputContract(lead);
    assertSafetyAndOutputContract(follower);
    assertEquals(InvertedValue.CounterClockwise_Positive, lead.MotorOutput.Inverted);
    assertEquals(InvertedValue.Clockwise_Positive, follower.MotorOutput.Inverted);
  }

  @Test
  void followerRequestUsesLeaderAlignmentAndControlFrequency() {
    var follower = HopperIOTalonFX.createFollowerRequest();

    assertEquals(17, follower.LeaderID);
    assertEquals(MotorAlignmentValue.Aligned, follower.MotorAlignment);
    assertEquals(100.0, follower.UpdateFreqHz, 1e-9);
  }

  @Test
  void statusFrequencyConfigPreservesMechanismTemperatureAndOptimizationRates() {
    var frequencies = HopperIOTalonFX.createStatusFrequencyConfig();

    assertEquals(50.0, frequencies.mechanismHz(), 1e-9);
    assertEquals(4.0, frequencies.temperatureHz(), 1e-9);
    assertEquals(4.0, frequencies.unspecifiedHz(), 1e-9);
  }

  private static void assertSafetyAndOutputContract(TalonFXConfiguration config) {
    assertTrue(config.CurrentLimits.StatorCurrentLimitEnable);
    assertEquals(80.0, config.CurrentLimits.StatorCurrentLimit, 1e-9);
    assertTrue(config.CurrentLimits.SupplyCurrentLimitEnable);
    assertEquals(40.0, config.CurrentLimits.SupplyCurrentLimit, 1e-9);
    assertEquals(20.0, config.CurrentLimits.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.2, config.CurrentLimits.SupplyCurrentLowerTime, 1e-9);
    assertEquals(12.0, config.Voltage.PeakForwardVoltage, 1e-9);
    assertEquals(-12.0, config.Voltage.PeakReverseVoltage, 1e-9);
    assertEquals(0.5, config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod, 1e-9);
    assertEquals(0.5, config.OpenLoopRamps.TorqueOpenLoopRampPeriod, 1e-9);
    assertEquals(0.5, config.OpenLoopRamps.VoltageOpenLoopRampPeriod, 1e-9);
    assertEquals(NeutralModeValue.Coast, config.MotorOutput.NeutralMode);
  }
}
