package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
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

/** Real TalonFX and CANcoder hardware interface for the intake. */
public class IntakeIOTalonFX implements IntakeIO {
  record StatusFrequencyConfig(double mechanismHz, double temperatureHz, double unspecifiedHz) {}

  private final TalonFX rollerLead = new TalonFX(ROLLER_LEAD_ID);
  private final TalonFX rollerFollower = new TalonFX(ROLLER_FOLLOWER_ID);
  private final TalonFX arm = new TalonFX(ARM_MOTOR_ID);
  private final CANcoder armCancoder = new CANcoder(ARM_CANCODER_ID);

  private final VoltageOut rollerVoltageRequest = createVoltageRequest();
  private final DutyCycleOut rollerDutyCycleRequest = createDutyCycleRequest();
  private final TorqueCurrentFOC rollerTorqueRequest = createRollerTorqueRequest();
  private final PositionVoltage armPositionRequest = createArmPositionRequest();
  private final VoltageOut armVoltageRequest = createVoltageRequest();
  private final DutyCycleOut armDutyCycleRequest = createDutyCycleRequest();
  private final TorqueCurrentFOC armTorqueRequest = createArmTorqueRequest();
  private final NeutralOut armNeutralRequest = createArmNeutralRequest();

  private final StatusSignal<Angle> rollerLeadPosition = rollerLead.getPosition();
  private final StatusSignal<AngularVelocity> rollerLeadVelocity = rollerLead.getVelocity();
  private final StatusSignal<Voltage> rollerLeadVoltage = rollerLead.getMotorVoltage();
  private final StatusSignal<Current> rollerLeadCurrent = rollerLead.getStatorCurrent();
  private final StatusSignal<Temperature> rollerLeadTemperature = rollerLead.getDeviceTemp();
  private final StatusSignal<Angle> rollerFollowerPosition = rollerFollower.getPosition();
  private final StatusSignal<AngularVelocity> rollerFollowerVelocity = rollerFollower.getVelocity();
  private final StatusSignal<Voltage> rollerFollowerVoltage = rollerFollower.getMotorVoltage();
  private final StatusSignal<Current> rollerFollowerCurrent = rollerFollower.getStatorCurrent();
  private final StatusSignal<Temperature> rollerFollowerTemperature =
      rollerFollower.getDeviceTemp();
  private final StatusSignal<Angle> armPosition = arm.getPosition();
  private final StatusSignal<AngularVelocity> armVelocity = arm.getVelocity();
  private final StatusSignal<Voltage> armVoltage = arm.getMotorVoltage();
  private final StatusSignal<Current> armCurrent = arm.getStatorCurrent();
  private final StatusSignal<Temperature> armTemperature = arm.getDeviceTemp();
  private final StatusSignal<Angle> armCancoderPosition = armCancoder.getPosition();
  private final StatusSignal<Angle> armCancoderAbsolutePosition = armCancoder.getAbsolutePosition();
  private final StatusSignal<AngularVelocity> armCancoderVelocity = armCancoder.getVelocity();

  private final BaseStatusSignal[] temperatureRefreshSignals =
      createTemperatureRefreshSignals(
          rollerLeadTemperature, rollerFollowerTemperature, armTemperature);
  private final BaseStatusSignal[] cancoderRefreshSignals =
      createCancoderRefreshSignals(
          armCancoderPosition, armCancoderAbsolutePosition, armCancoderVelocity);
  private final Debouncer rollerLeadConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rollerFollowerConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer armConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer armCancoderConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeIOTalonFX() {
    var statusFrequencies = createStatusFrequencyConfig();

    tryUntilOk(5, () -> armCancoder.getConfigurator().apply(createArmCancoderConfig(), 0.25));
    tryUntilOk(5, () -> rollerLead.getConfigurator().apply(createRollerLeadConfig(), 0.25));
    tryUntilOk(5, () -> rollerFollower.getConfigurator().apply(createRollerFollowerConfig(), 0.25));
    tryUntilOk(5, () -> arm.getConfigurator().apply(createArmConfig(), 0.25));

    tryUntilOk(5, () -> rollerLead.clearStickyFaults(0.25));
    tryUntilOk(5, () -> rollerFollower.clearStickyFaults(0.25));
    tryUntilOk(5, () -> arm.clearStickyFaults(0.25));
    tryUntilOk(5, () -> armCancoder.clearStickyFaults(0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        statusFrequencies.mechanismHz(),
        rollerLeadPosition,
        rollerLeadVelocity,
        rollerLeadVoltage,
        rollerLeadCurrent,
        rollerFollowerPosition,
        rollerFollowerVelocity,
        rollerFollowerVoltage,
        rollerFollowerCurrent,
        armPosition,
        armVelocity,
        armVoltage,
        armCurrent,
        armCancoderPosition,
        armCancoderAbsolutePosition,
        armCancoderVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        statusFrequencies.temperatureHz(),
        rollerLeadTemperature,
        rollerFollowerTemperature,
        armTemperature);
    ParentDevice.optimizeBusUtilizationForAll(
        statusFrequencies.unspecifiedHz(), rollerLead, rollerFollower, arm, armCancoder);

    tryUntilOk(5, () -> rollerFollower.setControl(createRollerFollowerRequest()));
  }

  static VoltageOut createVoltageRequest() {
    return new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(CONTROL_HZ);
  }

  static DutyCycleOut createDutyCycleRequest() {
    return new DutyCycleOut(0.0).withEnableFOC(true).withUpdateFreqHz(CONTROL_HZ);
  }

  static TorqueCurrentFOC createRollerTorqueRequest() {
    return new TorqueCurrentFOC(0.0)
        .withMaxAbsDutyCycle(ROLLER_STATE_MAX_DUTY)
        .withDeadband(Amps.of(1.0))
        .withOverrideCoastDurNeutral(true)
        .withUpdateFreqHz(CONTROL_HZ);
  }

  static TorqueCurrentFOC createArmTorqueRequest() {
    return new TorqueCurrentFOC(0.0)
        .withMaxAbsDutyCycle(1.0)
        .withDeadband(Amps.of(1.0))
        .withOverrideCoastDurNeutral(true)
        .withUpdateFreqHz(CONTROL_HZ);
  }

  static PositionVoltage createArmPositionRequest() {
    return new PositionVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(CONTROL_HZ);
  }

  static NeutralOut createArmNeutralRequest() {
    return new NeutralOut().withUpdateFreqHz(CONTROL_HZ);
  }

  static Follower createRollerFollowerRequest() {
    return new Follower(ROLLER_LEAD_ID, MotorAlignmentValue.Opposed).withUpdateFreqHz(CONTROL_HZ);
  }

  static StatusFrequencyConfig createStatusFrequencyConfig() {
    return new StatusFrequencyConfig(MECHANISM_HZ, SLOW_HZ, SLOW_HZ);
  }

  static BaseStatusSignal[] createTemperatureRefreshSignals(
      StatusSignal<Temperature> rollerLead,
      StatusSignal<Temperature> rollerFollower,
      StatusSignal<Temperature> arm) {
    return new BaseStatusSignal[] {rollerLead, rollerFollower, arm};
  }

  static BaseStatusSignal[] createCancoderRefreshSignals(
      StatusSignal<Angle> position,
      StatusSignal<Angle> absolutePosition,
      StatusSignal<AngularVelocity> velocity) {
    return new BaseStatusSignal[] {position, absolutePosition, velocity};
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var rollerLeadStatus =
        BaseStatusSignal.refreshAll(
            rollerLeadPosition, rollerLeadVelocity, rollerLeadVoltage, rollerLeadCurrent);
    var rollerFollowerStatus =
        BaseStatusSignal.refreshAll(
            rollerFollowerPosition,
            rollerFollowerVelocity,
            rollerFollowerVoltage,
            rollerFollowerCurrent);
    var armStatus = BaseStatusSignal.refreshAll(armPosition, armVelocity, armVoltage, armCurrent);
    BaseStatusSignal.refreshAll(temperatureRefreshSignals);
    var armCancoderStatus = BaseStatusSignal.refreshAll(cancoderRefreshSignals);

    inputs.rollerLeadConnected = rollerLeadConnectedDebouncer.calculate(rollerLeadStatus.isOK());
    inputs.rollerLeadPositionRotations = rollerLeadPosition.getValueAsDouble();
    inputs.rollerLeadVelocityRpm = rollerLeadVelocity.getValueAsDouble() * 60.0;
    inputs.rollerLeadAppliedVolts = rollerLeadVoltage.getValueAsDouble();
    inputs.rollerLeadCurrentAmps = rollerLeadCurrent.getValueAsDouble();
    inputs.rollerLeadTempCelsius = rollerLeadTemperature.getValueAsDouble();

    inputs.rollerFollowerConnected =
        rollerFollowerConnectedDebouncer.calculate(rollerFollowerStatus.isOK());
    inputs.rollerFollowerPositionRotations = rollerFollowerPosition.getValueAsDouble();
    inputs.rollerFollowerVelocityRpm = rollerFollowerVelocity.getValueAsDouble() * 60.0;
    inputs.rollerFollowerAppliedVolts = rollerFollowerVoltage.getValueAsDouble();
    inputs.rollerFollowerCurrentAmps = rollerFollowerCurrent.getValueAsDouble();
    inputs.rollerFollowerTempCelsius = rollerFollowerTemperature.getValueAsDouble();

    inputs.armConnected = armConnectedDebouncer.calculate(armStatus.isOK());
    inputs.armPositionRotations = armPosition.getValueAsDouble();
    inputs.armVelocityRpm = armVelocity.getValueAsDouble() * 60.0;
    inputs.armAppliedVolts = armVoltage.getValueAsDouble();
    inputs.armCurrentAmps = armCurrent.getValueAsDouble();
    inputs.armTempCelsius = armTemperature.getValueAsDouble();

    inputs.armCancoderConnected = armCancoderConnectedDebouncer.calculate(armCancoderStatus.isOK());
    inputs.armCancoderPositionRotations = armCancoderPosition.getValueAsDouble();
    inputs.armCancoderAbsolutePositionRotations = armCancoderAbsolutePosition.getValueAsDouble();
    inputs.armCancoderVelocityRpm = armCancoderVelocity.getValueAsDouble() * 60.0;
  }

  @Override
  public void setRollerVoltage(Voltage voltage) {
    rollerLead.setControl(
        rollerVoltageRequest.withOutput(
            MathUtil.clamp(
                voltage.in(Volts), -ROLLER_MAX_VOLTAGE.in(Volts), ROLLER_MAX_VOLTAGE.in(Volts))));
  }

  @Override
  public void setRollerDutyCycle(double output) {
    rollerLead.setControl(rollerDutyCycleRequest.withOutput(MathUtil.clamp(output, -1.0, 1.0)));
  }

  @Override
  public void setRollerTorqueCurrent(Current current, double maxAbsDutyCycle) {
    rollerLead.setControl(
        rollerTorqueRequest
            .withOutput(MathUtil.clamp(current.in(Amps), -80.0, 80.0))
            .withMaxAbsDutyCycle(MathUtil.clamp(maxAbsDutyCycle, 0.0, 1.0)));
  }

  @Override
  public void setArmPosition(double rotations, int slot) {
    arm.setControl(
        armPositionRequest
            .withPosition(MathUtil.clamp(rotations, ARM_DEPLOYED_ROTATIONS, ARM_STOWED_ROTATIONS))
            .withSlot(MathUtil.clamp(slot, ARM_FAST_SLOT, ARM_SLOW_SLOT)));
  }

  @Override
  public void setArmVoltage(Voltage voltage) {
    arm.setControl(
        armVoltageRequest.withOutput(
            MathUtil.clamp(
                voltage.in(Volts), -ARM_MAX_VOLTAGE.in(Volts), ARM_MAX_VOLTAGE.in(Volts))));
  }

  @Override
  public void setArmDutyCycle(double output) {
    arm.setControl(armDutyCycleRequest.withOutput(MathUtil.clamp(output, -1.0, 1.0)));
  }

  @Override
  public void setArmTorqueCurrent(Current current) {
    arm.setControl(armTorqueRequest.withOutput(MathUtil.clamp(current.in(Amps), -50.0, 50.0)));
  }

  @Override
  public void setArmBrakeNeutral() {
    arm.setControl(armNeutralRequest);
  }
}
