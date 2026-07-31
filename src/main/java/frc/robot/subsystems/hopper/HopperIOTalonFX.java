package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.hopper.HopperConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Real TalonFX hardware interface for the hopper lead and follower motors. */
public class HopperIOTalonFX implements HopperIO {
  record StatusFrequencyConfig(double mechanismHz, double temperatureHz, double unspecifiedHz) {}

  private final TalonFX leadMotor = new TalonFX(LEAD_MOTOR_ID);
  private final TalonFX followerMotor = new TalonFX(FOLLOWER_MOTOR_ID);
  private final VoltageOut voltageRequest = createVoltageRequest();
  private final DutyCycleOut dutyCycleRequest = createDutyCycleRequest();
  private final TorqueCurrentFOC torqueCurrentRequest = createTorqueCurrentRequest();

  private final StatusSignal<Angle> leadPosition = leadMotor.getPosition();
  private final StatusSignal<AngularVelocity> leadVelocity = leadMotor.getVelocity();
  private final StatusSignal<Voltage> leadVoltage = leadMotor.getMotorVoltage();
  private final StatusSignal<Current> leadCurrent = leadMotor.getStatorCurrent();
  private final StatusSignal<Temperature> leadTemperature = leadMotor.getDeviceTemp();
  private final StatusSignal<Angle> followerPosition = followerMotor.getPosition();
  private final StatusSignal<AngularVelocity> followerVelocity = followerMotor.getVelocity();
  private final StatusSignal<Voltage> followerVoltage = followerMotor.getMotorVoltage();
  private final StatusSignal<Current> followerCurrent = followerMotor.getStatorCurrent();
  private final StatusSignal<Temperature> followerTemperature = followerMotor.getDeviceTemp();
  private final BaseStatusSignal[] temperatureRefreshSignals =
      createTemperatureRefreshSignals(leadTemperature, followerTemperature);
  private final Debouncer leadConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public HopperIOTalonFX() {
    var statusFrequencies = createStatusFrequencyConfig();

    tryUntilOk(5, () -> leadMotor.getConfigurator().apply(createLeadConfig(), 0.25));
    tryUntilOk(5, () -> followerMotor.getConfigurator().apply(createFollowerConfig(), 0.25));
    tryUntilOk(5, () -> leadMotor.clearStickyFaults(0.25));
    tryUntilOk(5, () -> followerMotor.clearStickyFaults(0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        statusFrequencies.mechanismHz(),
        leadPosition,
        leadVelocity,
        leadVoltage,
        leadCurrent,
        followerPosition,
        followerVelocity,
        followerVoltage,
        followerCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(
        statusFrequencies.temperatureHz(), leadTemperature, followerTemperature);
    ParentDevice.optimizeBusUtilizationForAll(
        statusFrequencies.unspecifiedHz(), leadMotor, followerMotor);

    tryUntilOk(5, () -> followerMotor.setControl(createFollowerRequest()));
  }

  static VoltageOut createVoltageRequest() {
    return new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(CONTROL_UPDATE_HZ);
  }

  static DutyCycleOut createDutyCycleRequest() {
    return new DutyCycleOut(0.0).withEnableFOC(true).withUpdateFreqHz(CONTROL_UPDATE_HZ);
  }

  static TorqueCurrentFOC createTorqueCurrentRequest() {
    return new TorqueCurrentFOC(0.0)
        .withMaxAbsDutyCycle(MAX_ABS_DUTY_CYCLE)
        .withDeadband(TORQUE_DEADBAND)
        .withOverrideCoastDurNeutral(true)
        .withUpdateFreqHz(CONTROL_UPDATE_HZ);
  }

  static Follower createFollowerRequest() {
    return new Follower(LEAD_MOTOR_ID, MotorAlignmentValue.Aligned)
        .withUpdateFreqHz(CONTROL_UPDATE_HZ);
  }

  static StatusFrequencyConfig createStatusFrequencyConfig() {
    return new StatusFrequencyConfig(MECHANISM_STATUS_HZ, SLOW_STATUS_HZ, SLOW_STATUS_HZ);
  }

  static BaseStatusSignal[] createTemperatureRefreshSignals(
      StatusSignal<Temperature> leadTemperature, StatusSignal<Temperature> followerTemperature) {
    return new BaseStatusSignal[] {leadTemperature, followerTemperature};
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    var leadStatus =
        BaseStatusSignal.refreshAll(leadPosition, leadVelocity, leadVoltage, leadCurrent);
    var followerStatus =
        BaseStatusSignal.refreshAll(
            followerPosition, followerVelocity, followerVoltage, followerCurrent);
    BaseStatusSignal.refreshAll(temperatureRefreshSignals);

    inputs.leadConnected = leadConnectedDebouncer.calculate(leadStatus.isOK());
    inputs.leadPositionRotations = leadPosition.getValueAsDouble();
    inputs.leadVelocityRpm = leadVelocity.getValueAsDouble() * 60.0;
    inputs.leadAppliedVolts = leadVoltage.getValueAsDouble();
    inputs.leadCurrentAmps = leadCurrent.getValueAsDouble();
    inputs.leadTempCelsius = leadTemperature.getValueAsDouble();

    inputs.followerConnected = followerConnectedDebouncer.calculate(followerStatus.isOK());
    inputs.followerPositionRotations = followerPosition.getValueAsDouble();
    inputs.followerVelocityRpm = followerVelocity.getValueAsDouble() * 60.0;
    inputs.followerAppliedVolts = followerVoltage.getValueAsDouble();
    inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();
    inputs.followerTempCelsius = followerTemperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    leadMotor.setControl(
        voltageRequest.withOutput(
            MathUtil.clamp(voltage.in(Volts), -MAX_VOLTAGE.in(Volts), MAX_VOLTAGE.in(Volts))));
  }

  @Override
  public void setDutyCycle(double output) {
    leadMotor.setControl(dutyCycleRequest.withOutput(MathUtil.clamp(output, -1.0, 1.0)));
  }

  @Override
  public void setTorqueCurrent(Current current) {
    leadMotor.setControl(
        torqueCurrentRequest.withOutput(
            MathUtil.clamp(
                current.in(Amps), -STATOR_CURRENT_LIMIT.in(Amps), STATOR_CURRENT_LIMIT.in(Amps))));
  }
}
