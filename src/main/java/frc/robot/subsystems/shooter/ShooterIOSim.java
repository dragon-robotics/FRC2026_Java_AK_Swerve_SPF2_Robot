package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics simulation for the shooter flywheels, kicker, and hood. */
public class ShooterIOSim implements ShooterIO {
  private enum FlywheelControlMode {
    VOLTAGE,
    DUTY_CYCLE,
    TORQUE_CURRENT,
    VELOCITY
  }

  private enum KickerControlMode {
    VOLTAGE,
    DUTY_CYCLE,
    TORQUE_CURRENT
  }

  private enum HoodControlMode {
    VOLTAGE,
    DUTY_CYCLE,
    TORQUE_CURRENT,
    POSITION
  }

  private final DCMotorSim flywheelLeadSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(FLYWHEEL_SIM_MOTOR, FLYWHEEL_SIM_MOI, SIM_GEARING),
          FLYWHEEL_SIM_MOTOR);
  private final DCMotorSim flywheelFollowerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(FLYWHEEL_SIM_MOTOR, FLYWHEEL_SIM_MOI, SIM_GEARING),
          FLYWHEEL_SIM_MOTOR);
  private final DCMotorSim kickerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(KICKER_SIM_MOTOR, KICKER_SIM_MOI, SIM_GEARING),
          KICKER_SIM_MOTOR);
  private final DCMotorSim hoodSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(HOOD_SIM_MOTOR, HOOD_SIM_MOI, SIM_GEARING),
          HOOD_SIM_MOTOR);

  private FlywheelControlMode flywheelControlMode = FlywheelControlMode.VOLTAGE;
  private KickerControlMode kickerControlMode = KickerControlMode.VOLTAGE;
  private HoodControlMode hoodControlMode = HoodControlMode.VOLTAGE;

  private double requestedFlywheelRpm;
  private double requestedFlywheelVoltage;
  private double requestedFlywheelDutyCycle;
  private double requestedFlywheelTorqueCurrentAmps;
  private double requestedKickerVoltage;
  private double requestedKickerDutyCycle;
  private double requestedKickerTorqueCurrentAmps;
  private double requestedHoodRotations;
  private double requestedHoodVoltage;
  private double requestedHoodDutyCycle;
  private double requestedHoodTorqueCurrentAmps;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double flywheelLeadVolts = calculateFlywheelVoltage();
    double flywheelFollowerVolts = -flywheelLeadVolts;
    double kickerVolts = calculateKickerVoltage();
    double hoodVolts = calculateHoodVoltage();

    flywheelLeadSim.setInputVoltage(flywheelLeadVolts);
    flywheelFollowerSim.setInputVoltage(flywheelFollowerVolts);
    kickerSim.setInputVoltage(kickerVolts);
    hoodSim.setInputVoltage(hoodVolts);
    flywheelLeadSim.update(LOOP_PERIOD_SECONDS);
    flywheelFollowerSim.update(LOOP_PERIOD_SECONDS);
    kickerSim.update(LOOP_PERIOD_SECONDS);
    hoodSim.update(LOOP_PERIOD_SECONDS);

    inputs.flywheelLeadConnected = true;
    inputs.flywheelLeadPositionRotations = flywheelLeadSim.getAngularPositionRotations();
    inputs.flywheelLeadVelocityRpm = flywheelLeadSim.getAngularVelocityRPM();
    inputs.flywheelLeadAppliedVolts = flywheelLeadSim.getInputVoltage();
    inputs.flywheelLeadCurrentAmps = Math.abs(flywheelLeadSim.getCurrentDrawAmps());
    inputs.flywheelLeadTempCelsius = 0.0;

    inputs.flywheelFollowerConnected = true;
    inputs.flywheelFollowerPositionRotations = flywheelFollowerSim.getAngularPositionRotations();
    inputs.flywheelFollowerVelocityRpm = flywheelFollowerSim.getAngularVelocityRPM();
    inputs.flywheelFollowerAppliedVolts = flywheelFollowerSim.getInputVoltage();
    inputs.flywheelFollowerCurrentAmps = Math.abs(flywheelFollowerSim.getCurrentDrawAmps());
    inputs.flywheelFollowerTempCelsius = 0.0;

    inputs.kickerConnected = true;
    inputs.kickerPositionRotations = kickerSim.getAngularPositionRotations();
    inputs.kickerVelocityRpm = kickerSim.getAngularVelocityRPM();
    inputs.kickerAppliedVolts = kickerSim.getInputVoltage();
    inputs.kickerCurrentAmps = Math.abs(kickerSim.getCurrentDrawAmps());
    inputs.kickerTempCelsius = 0.0;

    inputs.hoodConnected = true;
    inputs.hoodPositionRotations = hoodSim.getAngularPositionRotations();
    inputs.hoodVelocityRpm = hoodSim.getAngularVelocityRPM();
    inputs.hoodAppliedVolts = hoodSim.getInputVoltage();
    inputs.hoodCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
    inputs.hoodTempCelsius = 0.0;
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    flywheelControlMode = FlywheelControlMode.VELOCITY;
    requestedFlywheelRpm = rpm;
  }

  @Override
  public void setFlywheelVoltage(Voltage voltage) {
    flywheelControlMode = FlywheelControlMode.VOLTAGE;
    requestedFlywheelVoltage =
        MathUtil.clamp(
            voltage.in(Volts), -FLYWHEEL_MAX_VOLTAGE.in(Volts), FLYWHEEL_MAX_VOLTAGE.in(Volts));
  }

  @Override
  public void setFlywheelDutyCycle(double output) {
    flywheelControlMode = FlywheelControlMode.DUTY_CYCLE;
    requestedFlywheelDutyCycle = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setFlywheelTorqueCurrent(Current current) {
    flywheelControlMode = FlywheelControlMode.TORQUE_CURRENT;
    requestedFlywheelTorqueCurrentAmps =
        MathUtil.clamp(
            current.in(Amps),
            FLYWHEEL_REVERSE_TORQUE_LIMIT.in(Amps),
            FLYWHEEL_STATOR_LIMIT.in(Amps));
  }

  @Override
  public void setKickerVoltage(Voltage voltage) {
    kickerControlMode = KickerControlMode.VOLTAGE;
    requestedKickerVoltage =
        MathUtil.clamp(
            voltage.in(Volts), -KICKER_MAX_VOLTAGE.in(Volts), KICKER_MAX_VOLTAGE.in(Volts));
  }

  @Override
  public void setKickerDutyCycle(double output) {
    kickerControlMode = KickerControlMode.DUTY_CYCLE;
    requestedKickerDutyCycle = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setKickerTorqueCurrent(Current current) {
    kickerControlMode = KickerControlMode.TORQUE_CURRENT;
    requestedKickerTorqueCurrentAmps =
        MathUtil.clamp(
            current.in(Amps), -KICKER_STATOR_LIMIT.in(Amps), KICKER_STATOR_LIMIT.in(Amps));
  }

  @Override
  public void setHoodPosition(double rotations) {
    hoodControlMode = HoodControlMode.POSITION;
    requestedHoodRotations = rotations;
  }

  @Override
  public void setHoodVoltage(Voltage voltage) {
    hoodControlMode = HoodControlMode.VOLTAGE;
    requestedHoodVoltage =
        MathUtil.clamp(voltage.in(Volts), -HOOD_MAX_VOLTAGE.in(Volts), HOOD_MAX_VOLTAGE.in(Volts));
  }

  @Override
  public void setHoodDutyCycle(double output) {
    hoodControlMode = HoodControlMode.DUTY_CYCLE;
    requestedHoodDutyCycle = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setHoodTorqueCurrent(Current current) {
    hoodControlMode = HoodControlMode.TORQUE_CURRENT;
    requestedHoodTorqueCurrentAmps =
        MathUtil.clamp(current.in(Amps), -HOOD_STATOR_LIMIT.in(Amps), HOOD_STATOR_LIMIT.in(Amps));
  }

  @Override
  public void resetHoodPosition(double rotations) {
    hoodSim.setState(Units.rotationsToRadians(rotations), 0.0);
  }

  private double calculateFlywheelVoltage() {
    return switch (flywheelControlMode) {
      case VOLTAGE -> requestedFlywheelVoltage;
      case DUTY_CYCLE -> requestedFlywheelDutyCycle * FLYWHEEL_MAX_VOLTAGE.in(Volts);
      case TORQUE_CURRENT -> torqueCurrentToVoltage(
          FLYWHEEL_SIM_MOTOR,
          flywheelLeadSim,
          requestedFlywheelTorqueCurrentAmps,
          FLYWHEEL_MAX_VOLTAGE.in(Volts));
      case VELOCITY -> {
        double feedforwardVolts =
            12.0
                * requestedFlywheelRpm
                / Units.radiansPerSecondToRotationsPerMinute(FLYWHEEL_SIM_MOTOR.freeSpeedRadPerSec);
        double feedbackVolts =
            0.0015 * (requestedFlywheelRpm - flywheelLeadSim.getAngularVelocityRPM());
        yield MathUtil.clamp(
            feedforwardVolts + feedbackVolts,
            -FLYWHEEL_MAX_VOLTAGE.in(Volts),
            FLYWHEEL_MAX_VOLTAGE.in(Volts));
      }
    };
  }

  private double calculateKickerVoltage() {
    return switch (kickerControlMode) {
      case VOLTAGE -> requestedKickerVoltage;
      case DUTY_CYCLE -> requestedKickerDutyCycle * KICKER_MAX_VOLTAGE.in(Volts);
      case TORQUE_CURRENT -> torqueCurrentToVoltage(
          KICKER_SIM_MOTOR,
          kickerSim,
          requestedKickerTorqueCurrentAmps,
          KICKER_MAX_VOLTAGE.in(Volts));
    };
  }

  private double calculateHoodVoltage() {
    return switch (hoodControlMode) {
      case VOLTAGE -> requestedHoodVoltage;
      case DUTY_CYCLE -> requestedHoodDutyCycle * HOOD_MAX_VOLTAGE.in(Volts);
      case TORQUE_CURRENT -> torqueCurrentToVoltage(
          HOOD_SIM_MOTOR, hoodSim, requestedHoodTorqueCurrentAmps, HOOD_MAX_VOLTAGE.in(Volts));
      case POSITION -> {
        double positionError = requestedHoodRotations - hoodSim.getAngularPositionRotations();
        yield MathUtil.clamp(
            8.0 * positionError - 0.02 * hoodSim.getAngularVelocityRPM(),
            -HOOD_MAX_VOLTAGE.in(Volts),
            HOOD_MAX_VOLTAGE.in(Volts));
      }
    };
  }

  private static double torqueCurrentToVoltage(
      DCMotor motor, DCMotorSim sim, double currentAmps, double maxVoltage) {
    return MathUtil.clamp(
        motor.getVoltage(
            motor.getTorque(currentAmps), sim.getAngularVelocityRadPerSec() * SIM_GEARING),
        -maxVoltage,
        maxVoltage);
  }
}
