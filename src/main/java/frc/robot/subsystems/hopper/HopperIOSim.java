package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hopper.HopperConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HopperIOSim implements HopperIO {
  private enum ControlMode {
    VOLTAGE,
    DUTY_CYCLE,
    TORQUE_CURRENT
  }

  private final DCMotorSim leadSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(SIM_MOTOR, SIM_MOI_KG_METERS_SQUARED, SIM_GEARING),
          SIM_MOTOR);
  private final DCMotorSim followerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(SIM_MOTOR, SIM_MOI_KG_METERS_SQUARED, SIM_GEARING),
          SIM_MOTOR);

  private ControlMode controlMode = ControlMode.VOLTAGE;
  private double requestedVoltage;
  private double requestedDutyCycle;
  private double requestedTorqueCurrentAmps;

  @Override
  public void setVoltage(Voltage voltage) {
    controlMode = ControlMode.VOLTAGE;
    requestedVoltage =
        MathUtil.clamp(voltage.in(Volts), -MAX_VOLTAGE.in(Volts), MAX_VOLTAGE.in(Volts));
  }

  @Override
  public void setDutyCycle(double output) {
    controlMode = ControlMode.DUTY_CYCLE;
    requestedDutyCycle = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setTorqueCurrent(Current current) {
    controlMode = ControlMode.TORQUE_CURRENT;
    requestedTorqueCurrentAmps =
        MathUtil.clamp(
            current.in(Amps), -STATOR_CURRENT_LIMIT.in(Amps), STATOR_CURRENT_LIMIT.in(Amps));
  }

  private double calculateVoltage(DCMotorSim sim) {
    return switch (controlMode) {
      case VOLTAGE -> requestedVoltage;
      case DUTY_CYCLE -> requestedDutyCycle * MAX_VOLTAGE.in(Volts);
      case TORQUE_CURRENT -> MathUtil.clamp(
          SIM_MOTOR.getVoltage(
              SIM_MOTOR.getTorque(requestedTorqueCurrentAmps),
              sim.getAngularVelocityRadPerSec() * SIM_GEARING),
          -MAX_VOLTAGE.in(Volts),
          MAX_VOLTAGE.in(Volts));
    };
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    double leadVolts = calculateVoltage(leadSim);
    double followerVolts = calculateVoltage(followerSim);
    leadSim.setInputVoltage(leadVolts);
    followerSim.setInputVoltage(followerVolts);
    leadSim.update(0.02);
    followerSim.update(0.02);

    inputs.leadConnected = true;
    inputs.leadPositionRotations = leadSim.getAngularPositionRotations();
    inputs.leadVelocityRpm = leadSim.getAngularVelocityRPM();
    inputs.leadAppliedVolts = leadVolts;
    inputs.leadCurrentAmps = Math.abs(leadSim.getCurrentDrawAmps());
    inputs.followerConnected = true;
    inputs.followerPositionRotations = followerSim.getAngularPositionRotations();
    inputs.followerVelocityRpm = followerSim.getAngularVelocityRPM();
    inputs.followerAppliedVolts = followerVolts;
    inputs.followerCurrentAmps = Math.abs(followerSim.getCurrentDrawAmps());
  }
}
