package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.junit.jupiter.api.Test;

class IntakeIOTalonFXConfigTest {
  @Test
  void rollerConfigsPreserveIdsDirectionLimitsAndFollowerAlignment() {
    var lead = IntakeConstants.createRollerLeadConfig();
    var follower = IntakeConstants.createRollerFollowerConfig();
    Follower request = IntakeIOTalonFX.createRollerFollowerRequest();

    assertEquals(21, IntakeConstants.ROLLER_LEAD_ID);
    assertEquals(20, IntakeConstants.ROLLER_FOLLOWER_ID);
    assertEquals(InvertedValue.Clockwise_Positive, lead.MotorOutput.Inverted);
    assertEquals(NeutralModeValue.Coast, lead.MotorOutput.NeutralMode);
    assertEquals(NeutralModeValue.Coast, follower.MotorOutput.NeutralMode);
    assertEquals(80.0, lead.CurrentLimits.StatorCurrentLimit, 1e-9);
    assertTrue(lead.CurrentLimits.StatorCurrentLimitEnable);
    assertEquals(40.0, lead.CurrentLimits.SupplyCurrentLimit, 1e-9);
    assertTrue(lead.CurrentLimits.SupplyCurrentLimitEnable);
    assertEquals(30.0, lead.CurrentLimits.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.2, lead.CurrentLimits.SupplyCurrentLowerTime, 1e-9);
    assertEquals(12.0, lead.Voltage.PeakForwardVoltage, 1e-9);
    assertEquals(-12.0, lead.Voltage.PeakReverseVoltage, 1e-9);
    assertTrue(follower.CurrentLimits.StatorCurrentLimitEnable);
    assertEquals(80.0, follower.CurrentLimits.StatorCurrentLimit, 1e-9);
    assertTrue(follower.CurrentLimits.SupplyCurrentLimitEnable);
    assertEquals(40.0, follower.CurrentLimits.SupplyCurrentLimit, 1e-9);
    assertEquals(30.0, follower.CurrentLimits.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.2, follower.CurrentLimits.SupplyCurrentLowerTime, 1e-9);
    assertEquals(12.0, follower.Voltage.PeakForwardVoltage, 1e-9);
    assertEquals(-12.0, follower.Voltage.PeakReverseVoltage, 1e-9);
    assertEquals(21, request.LeaderID);
    assertEquals(MotorAlignmentValue.Opposed, request.MotorAlignment);
    assertEquals(100.0, request.UpdateFreqHz, 1e-9);
  }

  @Test
  void armAndCancoderConfigsUseApprovedFusedFeedbackAndGains() {
    var arm = IntakeConstants.createArmConfig();
    var cancoder = IntakeConstants.createArmCancoderConfig();

    assertEquals(10, IntakeConstants.ARM_MOTOR_ID);
    assertEquals(0, IntakeConstants.ARM_CANCODER_ID);
    assertEquals(50.0, arm.CurrentLimits.StatorCurrentLimit, 1e-9);
    assertTrue(arm.CurrentLimits.StatorCurrentLimitEnable);
    assertEquals(30.0, arm.CurrentLimits.SupplyCurrentLimit, 1e-9);
    assertTrue(arm.CurrentLimits.SupplyCurrentLimitEnable);
    assertEquals(10.0, arm.Voltage.PeakForwardVoltage, 1e-9);
    assertEquals(-10.0, arm.Voltage.PeakReverseVoltage, 1e-9);
    assertEquals(NeutralModeValue.Brake, arm.MotorOutput.NeutralMode);
    assertEquals(InvertedValue.Clockwise_Positive, arm.MotorOutput.Inverted);
    assertEquals(0, arm.Feedback.FeedbackRemoteSensorID);
    assertEquals(FeedbackSensorSourceValue.FusedCANcoder, arm.Feedback.FeedbackSensorSource);
    assertEquals(1.0, arm.Feedback.SensorToMechanismRatio, 1e-9);
    assertEquals(40.0, arm.Feedback.RotorToSensorRatio, 1e-9);
    assertEquals(14.0, arm.Slot0.kP, 1e-9);
    assertEquals(8.0, arm.Slot1.kP, 1e-9);
    assertEquals(2.4, arm.Slot0.kV, 1e-9);
    assertEquals(2.4, arm.Slot1.kV, 1e-9);
    assertEquals(0.5, arm.Slot0.kG, 1e-9);
    assertEquals(0.5, arm.Slot1.kG, 1e-9);
    assertEquals(GravityTypeValue.Arm_Cosine, arm.Slot0.GravityType);
    assertEquals(GravityTypeValue.Arm_Cosine, arm.Slot1.GravityType);
    assertEquals(StaticFeedforwardSignValue.UseClosedLoopSign, arm.Slot0.StaticFeedforwardSign);
    assertEquals(StaticFeedforwardSignValue.UseClosedLoopSign, arm.Slot1.StaticFeedforwardSign);
    assertEquals(0.881064453125, cancoder.MagnetSensor.MagnetOffset, 1e-12);
    assertEquals(0.5, cancoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint, 1e-9);
    assertEquals(SensorDirectionValue.Clockwise_Positive, cancoder.MagnetSensor.SensorDirection);
  }

  @Test
  void requestsUseApprovedFocDutyDeadbandNeutralAndFrequencySettings() {
    VoltageOut voltage = IntakeIOTalonFX.createVoltageRequest();
    DutyCycleOut duty = IntakeIOTalonFX.createDutyCycleRequest();
    TorqueCurrentFOC rollerTorque = IntakeIOTalonFX.createRollerTorqueRequest();
    TorqueCurrentFOC armTorque = IntakeIOTalonFX.createArmTorqueRequest();
    PositionVoltage position = IntakeIOTalonFX.createArmPositionRequest();
    NeutralOut neutral = IntakeIOTalonFX.createArmNeutralRequest();

    assertTrue(voltage.EnableFOC);
    assertTrue(duty.EnableFOC);
    assertTrue(position.EnableFOC);
    assertEquals(1.0, rollerTorque.Deadband, 1e-9);
    assertEquals(0.80, rollerTorque.MaxAbsDutyCycle, 1e-9);
    assertTrue(rollerTorque.OverrideCoastDurNeutral);
    assertEquals(1.0, armTorque.Deadband, 1e-9);
    assertEquals(1.0, armTorque.MaxAbsDutyCycle, 1e-9);
    assertTrue(armTorque.OverrideCoastDurNeutral);
    assertEquals(100.0, voltage.UpdateFreqHz, 1e-9);
    assertEquals(100.0, duty.UpdateFreqHz, 1e-9);
    assertEquals(100.0, rollerTorque.UpdateFreqHz, 1e-9);
    assertEquals(100.0, armTorque.UpdateFreqHz, 1e-9);
    assertEquals(100.0, position.UpdateFreqHz, 1e-9);
    assertEquals(100.0, neutral.UpdateFreqHz, 1e-9);
  }

  @Test
  void statusFrequenciesUseApprovedCanSchedule() {
    var frequencies = IntakeIOTalonFX.createStatusFrequencyConfig();
    assertEquals(50.0, frequencies.mechanismHz(), 1e-9);
    assertEquals(4.0, frequencies.temperatureHz(), 1e-9);
    assertEquals(4.0, frequencies.unspecifiedHz(), 1e-9);
  }

  @Test
  void refreshGroupsPreserveTemperatureAndCancoderSignalOrder() {
    StatusSignal<Temperature> leadTemperature = signal(Temperature.class, Celsius::of);
    StatusSignal<Temperature> followerTemperature = signal(Temperature.class, Celsius::of);
    StatusSignal<Temperature> armTemperature = signal(Temperature.class, Celsius::of);
    StatusSignal<Angle> position = signal(Angle.class, Rotations::of);
    StatusSignal<Angle> absolutePosition = signal(Angle.class, Rotations::of);
    StatusSignal<AngularVelocity> velocity = signal(AngularVelocity.class, RPM::of);

    assertArrayEquals(
        new BaseStatusSignal[] {leadTemperature, followerTemperature, armTemperature},
        IntakeIOTalonFX.createTemperatureRefreshSignals(
            leadTemperature, followerTemperature, armTemperature));
    assertArrayEquals(
        new BaseStatusSignal[] {position, absolutePosition, velocity},
        IntakeIOTalonFX.createCancoderRefreshSignals(position, absolutePosition, velocity));
  }

  @Test
  void followerSourceGroupIncludesEveryRequiredLeaderOutputSignalInOrder() {
    StatusSignal<Voltage> voltage = signal(Voltage.class, Volts::of);
    StatusSignal<Double> dutyCycle = signal(Double.class, value -> value);
    StatusSignal<Current> torqueCurrent = signal(Current.class, Amps::of);

    assertArrayEquals(
        new BaseStatusSignal[] {voltage, dutyCycle, torqueCurrent},
        IntakeIOTalonFX.createFollowerSourceSignals(voltage, dutyCycle, torqueCurrent));
  }

  private static <T> StatusSignal<T> signal(
      Class<T> valueClass, java.util.function.DoubleFunction<T> converter) {
    return new StatusSignal<>(StatusCode.StatusCodeNotInitialized, valueClass, converter);
  }
}
