package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Celsius;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Temperature;
import org.junit.jupiter.api.Test;

class ShooterIOTalonFXConfigTest {
  @Test
  void flywheelVelocityRequestUsesSupportedLockedSettings() {
    VelocityTorqueCurrentFOC request = ShooterIOTalonFX.createFlywheelVelocityRequest();
    assertTrue(request.OverrideCoastDurNeutral);
    assertEquals(100.0, request.UpdateFreqHz, 1e-9);
  }

  @Test
  void directRequestsMatchHopperSettings() {
    VoltageOut voltage = ShooterIOTalonFX.createVoltageRequest();
    DutyCycleOut duty = ShooterIOTalonFX.createDutyCycleRequest();
    TorqueCurrentFOC torque = ShooterIOTalonFX.createTorqueCurrentRequest();
    assertTrue(voltage.EnableFOC);
    assertTrue(duty.EnableFOC);
    assertEquals(100.0, voltage.UpdateFreqHz, 1e-9);
    assertEquals(100.0, duty.UpdateFreqHz, 1e-9);
    assertEquals(1.0, torque.MaxAbsDutyCycle, 1e-9);
    assertEquals(1.0, torque.Deadband, 1e-9);
    assertTrue(torque.OverrideCoastDurNeutral);
    assertEquals(100.0, torque.UpdateFreqHz, 1e-9);
  }

  @Test
  void hoodPositionAndFollowerRequestsUseLockedSettings() {
    PositionVoltage hood = ShooterIOTalonFX.createHoodPositionRequest();
    Follower follower = ShooterIOTalonFX.createFlywheelFollowerRequest();
    assertTrue(hood.EnableFOC);
    assertEquals(100.0, hood.UpdateFreqHz, 1e-9);
    assertEquals(15, follower.LeaderID);
    assertEquals(MotorAlignmentValue.Opposed, follower.MotorAlignment);
    assertEquals(100.0, follower.UpdateFreqHz, 1e-9);
  }

  @Test
  void configsPreserveCurrentVoltageNeutralAndGains() {
    var lead = ShooterConstants.createFlywheelLeadConfig();
    var follower = ShooterConstants.createFlywheelFollowerConfig();
    var kicker = ShooterConstants.createKickerConfig();
    var hood = ShooterConstants.createHoodConfig();

    assertEquals(100.0, lead.CurrentLimits.StatorCurrentLimit, 1e-9);
    assertEquals(40.0, lead.CurrentLimits.SupplyCurrentLimit, 1e-9);
    assertEquals(20.0, lead.CurrentLimits.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.25, lead.CurrentLimits.SupplyCurrentLowerTime, 1e-9);
    assertEquals(120.0, lead.TorqueCurrent.PeakForwardTorqueCurrent, 1e-9);
    assertEquals(-40.0, lead.TorqueCurrent.PeakReverseTorqueCurrent, 1e-9);
    assertEquals(8.0, lead.Slot0.kP, 1e-9);
    assertEquals(4.325, lead.Slot0.kS, 1e-9);
    assertEquals(0.013, lead.Slot0.kV, 1e-9);
    assertEquals(InvertedValue.Clockwise_Positive, lead.MotorOutput.Inverted);
    assertEquals(NeutralModeValue.Coast, follower.MotorOutput.NeutralMode);

    assertEquals(80.0, kicker.CurrentLimits.StatorCurrentLimit, 1e-9);
    assertEquals(40.0, kicker.CurrentLimits.SupplyCurrentLimit, 1e-9);
    assertEquals(30.0, hood.CurrentLimits.StatorCurrentLimit, 1e-9);
    assertEquals(15.0, hood.CurrentLimits.SupplyCurrentLimit, 1e-9);
    assertEquals(10.0, hood.Voltage.PeakForwardVoltage, 1e-9);
    assertEquals(8.0, hood.Slot0.kP, 1e-9);
    assertEquals(0.1, hood.Slot0.kD, 1e-9);
    assertEquals(0.4, hood.Slot0.kG, 1e-9);
    assertEquals(NeutralModeValue.Brake, hood.MotorOutput.NeutralMode);
  }

  @Test
  void statusFrequenciesUseApprovedCanSchedule() {
    var frequencies = ShooterIOTalonFX.createStatusFrequencyConfig();
    assertEquals(50.0, frequencies.mechanismHz(), 1e-9);
    assertEquals(4.0, frequencies.temperatureHz(), 1e-9);
    assertEquals(4.0, frequencies.unspecifiedHz(), 1e-9);
  }

  @Test
  void temperatureRefreshGroupContainsAllFourSignals() {
    StatusSignal<Temperature> lead =
        new StatusSignal<>(StatusCode.StatusCodeNotInitialized, Temperature.class, Celsius::of);
    StatusSignal<Temperature> follower =
        new StatusSignal<>(StatusCode.StatusCodeNotInitialized, Temperature.class, Celsius::of);
    StatusSignal<Temperature> kicker =
        new StatusSignal<>(StatusCode.StatusCodeNotInitialized, Temperature.class, Celsius::of);
    StatusSignal<Temperature> hood =
        new StatusSignal<>(StatusCode.StatusCodeNotInitialized, Temperature.class, Celsius::of);

    assertArrayEquals(
        new BaseStatusSignal[] {lead, follower, kicker, hood},
        ShooterIOTalonFX.createTemperatureRefreshSignals(lead, follower, kicker, hood));
  }
}
