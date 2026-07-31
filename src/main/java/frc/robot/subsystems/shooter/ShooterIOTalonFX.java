package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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

/** Real TalonFX hardware interface for the shooter flywheels, kicker, and hood. */
public class ShooterIOTalonFX implements ShooterIO {
  record StatusFrequencyConfig(double mechanismHz, double temperatureHz, double unspecifiedHz) {}

  private final TalonFX flywheelLeadMotor = new TalonFX(FLYWHEEL_LEAD_MOTOR_ID);
  private final TalonFX flywheelFollowerMotor = new TalonFX(FLYWHEEL_FOLLOWER_MOTOR_ID);
  private final TalonFX kickerMotor = new TalonFX(KICKER_MOTOR_ID);
  private final TalonFX hoodMotor = new TalonFX(HOOD_MOTOR_ID);

  private final VelocityTorqueCurrentFOC flywheelVelocityRequest = createFlywheelVelocityRequest();
  private final VoltageOut flywheelVoltageRequest = createVoltageRequest();
  private final DutyCycleOut flywheelDutyCycleRequest = createDutyCycleRequest();
  private final TorqueCurrentFOC flywheelTorqueRequest = createTorqueCurrentRequest();
  private final VoltageOut kickerVoltageRequest = createVoltageRequest();
  private final DutyCycleOut kickerDutyCycleRequest = createDutyCycleRequest();
  private final TorqueCurrentFOC kickerTorqueRequest = createTorqueCurrentRequest();
  private final PositionVoltage hoodPositionRequest = createHoodPositionRequest();
  private final VoltageOut hoodVoltageRequest = createVoltageRequest();
  private final DutyCycleOut hoodDutyCycleRequest = createDutyCycleRequest();
  private final TorqueCurrentFOC hoodTorqueRequest = createTorqueCurrentRequest();

  private final StatusSignal<Angle> flywheelLeadPosition = flywheelLeadMotor.getPosition();
  private final StatusSignal<AngularVelocity> flywheelLeadVelocity =
      flywheelLeadMotor.getVelocity();
  private final StatusSignal<Voltage> flywheelLeadVoltage = flywheelLeadMotor.getMotorVoltage();
  private final StatusSignal<Current> flywheelLeadCurrent = flywheelLeadMotor.getStatorCurrent();
  private final StatusSignal<Temperature> flywheelLeadTemperature =
      flywheelLeadMotor.getDeviceTemp();
  private final StatusSignal<Angle> flywheelFollowerPosition = flywheelFollowerMotor.getPosition();
  private final StatusSignal<AngularVelocity> flywheelFollowerVelocity =
      flywheelFollowerMotor.getVelocity();
  private final StatusSignal<Voltage> flywheelFollowerVoltage =
      flywheelFollowerMotor.getMotorVoltage();
  private final StatusSignal<Current> flywheelFollowerCurrent =
      flywheelFollowerMotor.getStatorCurrent();
  private final StatusSignal<Temperature> flywheelFollowerTemperature =
      flywheelFollowerMotor.getDeviceTemp();
  private final StatusSignal<Angle> kickerPosition = kickerMotor.getPosition();
  private final StatusSignal<AngularVelocity> kickerVelocity = kickerMotor.getVelocity();
  private final StatusSignal<Voltage> kickerVoltage = kickerMotor.getMotorVoltage();
  private final StatusSignal<Current> kickerCurrent = kickerMotor.getStatorCurrent();
  private final StatusSignal<Temperature> kickerTemperature = kickerMotor.getDeviceTemp();
  private final StatusSignal<Angle> hoodPosition = hoodMotor.getPosition();
  private final StatusSignal<AngularVelocity> hoodVelocity = hoodMotor.getVelocity();
  private final StatusSignal<Voltage> hoodVoltage = hoodMotor.getMotorVoltage();
  private final StatusSignal<Current> hoodCurrent = hoodMotor.getStatorCurrent();
  private final StatusSignal<Temperature> hoodTemperature = hoodMotor.getDeviceTemp();

  private final BaseStatusSignal[] temperatureRefreshSignals =
      createTemperatureRefreshSignals(
          flywheelLeadTemperature, flywheelFollowerTemperature, kickerTemperature, hoodTemperature);
  private final Debouncer flywheelLeadConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer flywheelFollowerConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer kickerConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer hoodConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ShooterIOTalonFX() {
    var statusFrequencies = createStatusFrequencyConfig();

    tryUntilOk(
        5, () -> flywheelLeadMotor.getConfigurator().apply(createFlywheelLeadConfig(), 0.25));
    tryUntilOk(
        5,
        () -> flywheelFollowerMotor.getConfigurator().apply(createFlywheelFollowerConfig(), 0.25));
    tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(createKickerConfig(), 0.25));
    tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(createHoodConfig(), 0.25));

    tryUntilOk(5, () -> flywheelLeadMotor.clearStickyFaults(0.25));
    tryUntilOk(5, () -> flywheelFollowerMotor.clearStickyFaults(0.25));
    tryUntilOk(5, () -> kickerMotor.clearStickyFaults(0.25));
    tryUntilOk(5, () -> hoodMotor.clearStickyFaults(0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        statusFrequencies.mechanismHz(),
        flywheelLeadPosition,
        flywheelLeadVelocity,
        flywheelLeadVoltage,
        flywheelLeadCurrent,
        flywheelFollowerPosition,
        flywheelFollowerVelocity,
        flywheelFollowerVoltage,
        flywheelFollowerCurrent,
        kickerPosition,
        kickerVelocity,
        kickerVoltage,
        kickerCurrent,
        hoodPosition,
        hoodVelocity,
        hoodVoltage,
        hoodCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(
        statusFrequencies.temperatureHz(),
        flywheelLeadTemperature,
        flywheelFollowerTemperature,
        kickerTemperature,
        hoodTemperature);
    ParentDevice.optimizeBusUtilizationForAll(
        statusFrequencies.unspecifiedHz(),
        flywheelLeadMotor,
        flywheelFollowerMotor,
        kickerMotor,
        hoodMotor);

    tryUntilOk(5, () -> flywheelFollowerMotor.setControl(createFlywheelFollowerRequest()));
  }

  static VelocityTorqueCurrentFOC createFlywheelVelocityRequest() {
    return new VelocityTorqueCurrentFOC(0.0)
        .withOverrideCoastDurNeutral(true)
        .withUpdateFreqHz(100.0);
  }

  static PositionVoltage createHoodPositionRequest() {
    return new PositionVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(100.0);
  }

  static VoltageOut createVoltageRequest() {
    return new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(100.0);
  }

  static DutyCycleOut createDutyCycleRequest() {
    return new DutyCycleOut(0.0).withEnableFOC(true).withUpdateFreqHz(100.0);
  }

  static TorqueCurrentFOC createTorqueCurrentRequest() {
    return new TorqueCurrentFOC(0.0)
        .withMaxAbsDutyCycle(1.0)
        .withDeadband(Amps.of(1.0))
        .withOverrideCoastDurNeutral(true)
        .withUpdateFreqHz(100.0);
  }

  static Follower createFlywheelFollowerRequest() {
    return new Follower(FLYWHEEL_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)
        .withUpdateFreqHz(100.0);
  }

  static StatusFrequencyConfig createStatusFrequencyConfig() {
    return new StatusFrequencyConfig(50.0, 4.0, 4.0);
  }

  static BaseStatusSignal[] createTemperatureRefreshSignals(
      StatusSignal<Temperature> lead,
      StatusSignal<Temperature> follower,
      StatusSignal<Temperature> kicker,
      StatusSignal<Temperature> hood) {
    return new BaseStatusSignal[] {lead, follower, kicker, hood};
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var flywheelLeadStatus =
        BaseStatusSignal.refreshAll(
            flywheelLeadPosition, flywheelLeadVelocity, flywheelLeadVoltage, flywheelLeadCurrent);
    var flywheelFollowerStatus =
        BaseStatusSignal.refreshAll(
            flywheelFollowerPosition,
            flywheelFollowerVelocity,
            flywheelFollowerVoltage,
            flywheelFollowerCurrent);
    var kickerStatus =
        BaseStatusSignal.refreshAll(kickerPosition, kickerVelocity, kickerVoltage, kickerCurrent);
    var hoodStatus =
        BaseStatusSignal.refreshAll(hoodPosition, hoodVelocity, hoodVoltage, hoodCurrent);
    BaseStatusSignal.refreshAll(temperatureRefreshSignals);

    inputs.flywheelLeadConnected =
        flywheelLeadConnectedDebouncer.calculate(flywheelLeadStatus.isOK());
    inputs.flywheelLeadPositionRotations = flywheelLeadPosition.getValueAsDouble();
    inputs.flywheelLeadVelocityRpm = flywheelLeadVelocity.getValueAsDouble() * 60.0;
    inputs.flywheelLeadAppliedVolts = flywheelLeadVoltage.getValueAsDouble();
    inputs.flywheelLeadCurrentAmps = flywheelLeadCurrent.getValueAsDouble();
    inputs.flywheelLeadTempCelsius = flywheelLeadTemperature.getValueAsDouble();

    inputs.flywheelFollowerConnected =
        flywheelFollowerConnectedDebouncer.calculate(flywheelFollowerStatus.isOK());
    inputs.flywheelFollowerPositionRotations = flywheelFollowerPosition.getValueAsDouble();
    inputs.flywheelFollowerVelocityRpm = flywheelFollowerVelocity.getValueAsDouble() * 60.0;
    inputs.flywheelFollowerAppliedVolts = flywheelFollowerVoltage.getValueAsDouble();
    inputs.flywheelFollowerCurrentAmps = flywheelFollowerCurrent.getValueAsDouble();
    inputs.flywheelFollowerTempCelsius = flywheelFollowerTemperature.getValueAsDouble();

    inputs.kickerConnected = kickerConnectedDebouncer.calculate(kickerStatus.isOK());
    inputs.kickerPositionRotations = kickerPosition.getValueAsDouble();
    inputs.kickerVelocityRpm = kickerVelocity.getValueAsDouble() * 60.0;
    inputs.kickerAppliedVolts = kickerVoltage.getValueAsDouble();
    inputs.kickerCurrentAmps = kickerCurrent.getValueAsDouble();
    inputs.kickerTempCelsius = kickerTemperature.getValueAsDouble();

    inputs.hoodConnected = hoodConnectedDebouncer.calculate(hoodStatus.isOK());
    inputs.hoodPositionRotations = hoodPosition.getValueAsDouble();
    inputs.hoodVelocityRpm = hoodVelocity.getValueAsDouble() * 60.0;
    inputs.hoodAppliedVolts = hoodVoltage.getValueAsDouble();
    inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();
    inputs.hoodTempCelsius = hoodTemperature.getValueAsDouble();
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    flywheelLeadMotor.setControl(flywheelVelocityRequest.withVelocity(rpm / 60.0));
  }

  @Override
  public void setFlywheelVoltage(Voltage voltage) {
    flywheelLeadMotor.setControl(
        flywheelVoltageRequest.withOutput(
            MathUtil.clamp(
                voltage.in(Volts),
                -FLYWHEEL_MAX_VOLTAGE.in(Volts),
                FLYWHEEL_MAX_VOLTAGE.in(Volts))));
  }

  @Override
  public void setFlywheelDutyCycle(double output) {
    flywheelLeadMotor.setControl(
        flywheelDutyCycleRequest.withOutput(MathUtil.clamp(output, -1.0, 1.0)));
  }

  @Override
  public void setFlywheelTorqueCurrent(Current current) {
    flywheelLeadMotor.setControl(
        flywheelTorqueRequest.withOutput(
            MathUtil.clamp(
                current.in(Amps),
                FLYWHEEL_REVERSE_TORQUE_LIMIT.in(Amps),
                FLYWHEEL_STATOR_LIMIT.in(Amps))));
  }

  @Override
  public void setKickerVoltage(Voltage voltage) {
    kickerMotor.setControl(
        kickerVoltageRequest.withOutput(
            MathUtil.clamp(
                voltage.in(Volts), -KICKER_MAX_VOLTAGE.in(Volts), KICKER_MAX_VOLTAGE.in(Volts))));
  }

  @Override
  public void setKickerDutyCycle(double output) {
    kickerMotor.setControl(kickerDutyCycleRequest.withOutput(MathUtil.clamp(output, -1.0, 1.0)));
  }

  @Override
  public void setKickerTorqueCurrent(Current current) {
    kickerMotor.setControl(
        kickerTorqueRequest.withOutput(
            MathUtil.clamp(
                current.in(Amps), -KICKER_STATOR_LIMIT.in(Amps), KICKER_STATOR_LIMIT.in(Amps))));
  }

  @Override
  public void setHoodPosition(double rotations) {
    hoodMotor.setControl(hoodPositionRequest.withPosition(rotations));
  }

  @Override
  public void setHoodVoltage(Voltage voltage) {
    hoodMotor.setControl(
        hoodVoltageRequest.withOutput(
            MathUtil.clamp(
                voltage.in(Volts), -HOOD_MAX_VOLTAGE.in(Volts), HOOD_MAX_VOLTAGE.in(Volts))));
  }

  @Override
  public void setHoodDutyCycle(double output) {
    hoodMotor.setControl(hoodDutyCycleRequest.withOutput(MathUtil.clamp(output, -1.0, 1.0)));
  }

  @Override
  public void setHoodTorqueCurrent(Current current) {
    hoodMotor.setControl(
        hoodTorqueRequest.withOutput(
            MathUtil.clamp(
                current.in(Amps), -HOOD_STATOR_LIMIT.in(Amps), HOOD_STATOR_LIMIT.in(Amps))));
  }

  @Override
  public void resetHoodPosition(double rotations) {
    tryUntilOk(5, () -> hoodMotor.setPosition(rotations, 0.25));
  }
}
