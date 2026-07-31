package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Physics simulation for the intake rollers, arm motor, and arm CANcoder. */
public class IntakeIOSim implements IntakeIO {
  private enum RollerControlMode {
    VOLTAGE,
    DUTY_CYCLE,
    TORQUE_CURRENT
  }

  private enum ArmControlMode {
    POSITION,
    VOLTAGE,
    DUTY_CYCLE,
    TORQUE_CURRENT,
    BRAKE
  }

  private final DCMotorSim rollerLeadSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(ROLLER_SIM_MOTOR, ROLLER_SIM_MOI, ROLLER_SIM_GEARING),
          ROLLER_SIM_MOTOR);
  private final DCMotorSim rollerFollowerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(ROLLER_SIM_MOTOR, ROLLER_SIM_MOI, ROLLER_SIM_GEARING),
          ROLLER_SIM_MOTOR);
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          ARM_SIM_MOTOR,
          ARM_GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, ARM_MASS_KG),
          ARM_LENGTH_METERS,
          ARM_MIN_RADIANS,
          ARM_MAX_RADIANS,
          true,
          ARM_START_RADIANS);

  private RollerControlMode rollerControlMode = RollerControlMode.VOLTAGE;
  private ArmControlMode armControlMode = ArmControlMode.BRAKE;

  private double requestedRollerVoltage;
  private double requestedRollerDutyCycle;
  private double requestedRollerTorqueCurrentAmps;
  private double requestedRollerMaxDuty;
  private double requestedArmRotations;
  private int requestedArmSlot;
  private double requestedArmVoltage;
  private double requestedArmDutyCycle;
  private double requestedArmTorqueCurrentAmps;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double leadRotorSpeed = rollerLeadSim.getAngularVelocityRadPerSec() * ROLLER_SIM_GEARING;
    double followerRotorSpeed =
        rollerFollowerSim.getAngularVelocityRadPerSec() * ROLLER_SIM_GEARING;
    double armRotorSpeed = armSim.getVelocityRadPerSec() * ARM_GEAR_RATIO;

    double leadVolts = calculateRollerVoltage(leadRotorSpeed);
    double followerVolts = -leadVolts;
    double armVolts = calculateArmVoltage(armRotorSpeed);

    leadVolts =
        limitVoltageForStatorCurrent(
            ROLLER_SIM_MOTOR, leadRotorSpeed, leadVolts, ROLLER_STATOR_LIMIT.in(Amps));
    followerVolts =
        limitVoltageForStatorCurrent(
            ROLLER_SIM_MOTOR, followerRotorSpeed, followerVolts, ROLLER_STATOR_LIMIT.in(Amps));
    armVolts =
        limitVoltageForStatorCurrent(
            ARM_SIM_MOTOR, armRotorSpeed, armVolts, ARM_STATOR_LIMIT.in(Amps));

    double estimatedRollerLeadCurrentAmps =
        Math.min(
            ROLLER_STATOR_LIMIT.in(Amps),
            Math.abs(ROLLER_SIM_MOTOR.getCurrent(leadRotorSpeed, leadVolts)));
    double estimatedRollerFollowerCurrentAmps =
        Math.min(
            ROLLER_STATOR_LIMIT.in(Amps),
            Math.abs(ROLLER_SIM_MOTOR.getCurrent(followerRotorSpeed, followerVolts)));
    double estimatedArmCurrentAmps =
        Math.min(
            ARM_STATOR_LIMIT.in(Amps), Math.abs(ARM_SIM_MOTOR.getCurrent(armRotorSpeed, armVolts)));
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            estimatedRollerLeadCurrentAmps,
            estimatedRollerFollowerCurrentAmps,
            estimatedArmCurrentAmps));

    double loadedVoltage = RoboRioSim.getVInVoltage();
    double rollerVoltageLimit =
        loadedVoltage
            * (rollerControlMode == RollerControlMode.TORQUE_CURRENT
                ? requestedRollerMaxDuty
                : 1.0);
    leadVolts = MathUtil.clamp(leadVolts, -rollerVoltageLimit, rollerVoltageLimit);
    followerVolts = MathUtil.clamp(followerVolts, -rollerVoltageLimit, rollerVoltageLimit);
    double armVoltageLimit = Math.min(loadedVoltage, ARM_MAX_VOLTAGE.in(Volts));
    armVolts = MathUtil.clamp(armVolts, -armVoltageLimit, armVoltageLimit);

    rollerLeadSim.setInputVoltage(leadVolts);
    rollerFollowerSim.setInputVoltage(followerVolts);
    armSim.setInputVoltage(armVolts);
    rollerLeadSim.update(LOOP_PERIOD_SECONDS);
    rollerFollowerSim.update(LOOP_PERIOD_SECONDS);
    armSim.update(LOOP_PERIOD_SECONDS);

    inputs.rollerLeadConnected = true;
    inputs.rollerLeadPositionRotations = rollerLeadSim.getAngularPositionRotations();
    inputs.rollerLeadVelocityRpm = rollerLeadSim.getAngularVelocityRPM();
    inputs.rollerLeadAppliedVolts = rollerLeadSim.getInputVoltage();
    inputs.rollerLeadCurrentAmps =
        Math.min(ROLLER_STATOR_LIMIT.in(Amps), Math.abs(rollerLeadSim.getCurrentDrawAmps()));
    inputs.rollerLeadTempCelsius = 0.0;

    inputs.rollerFollowerConnected = true;
    inputs.rollerFollowerPositionRotations = rollerFollowerSim.getAngularPositionRotations();
    inputs.rollerFollowerVelocityRpm = rollerFollowerSim.getAngularVelocityRPM();
    inputs.rollerFollowerAppliedVolts = rollerFollowerSim.getInputVoltage();
    inputs.rollerFollowerCurrentAmps =
        Math.min(ROLLER_STATOR_LIMIT.in(Amps), Math.abs(rollerFollowerSim.getCurrentDrawAmps()));
    inputs.rollerFollowerTempCelsius = 0.0;

    double armPositionRotations = Units.radiansToRotations(armSim.getAngleRads());
    double armVelocityRpm =
        Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec());
    inputs.armConnected = true;
    inputs.armPositionRotations = armPositionRotations;
    inputs.armVelocityRpm = armVelocityRpm;
    inputs.armAppliedVolts = armSim.getInput(0);
    inputs.armCurrentAmps =
        Math.min(ARM_STATOR_LIMIT.in(Amps), Math.abs(armSim.getCurrentDrawAmps()));
    inputs.armTempCelsius = 0.0;

    inputs.armCancoderConnected = true;
    inputs.armCancoderPositionRotations = armPositionRotations;
    inputs.armCancoderAbsolutePositionRotations =
        MathUtil.inputModulus(armPositionRotations, -0.5, 0.5);
    inputs.armCancoderVelocityRpm = armVelocityRpm;
  }

  @Override
  public void setRollerVoltage(Voltage voltage) {
    rollerControlMode = RollerControlMode.VOLTAGE;
    requestedRollerVoltage =
        MathUtil.clamp(
            voltage.in(Volts), -ROLLER_MAX_VOLTAGE.in(Volts), ROLLER_MAX_VOLTAGE.in(Volts));
  }

  @Override
  public void setRollerDutyCycle(double output) {
    rollerControlMode = RollerControlMode.DUTY_CYCLE;
    requestedRollerDutyCycle = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setRollerTorqueCurrent(Current current, double maxAbsDutyCycle) {
    rollerControlMode = RollerControlMode.TORQUE_CURRENT;
    requestedRollerTorqueCurrentAmps =
        MathUtil.clamp(
            current.in(Amps), -ROLLER_STATOR_LIMIT.in(Amps), ROLLER_STATOR_LIMIT.in(Amps));
    requestedRollerMaxDuty = MathUtil.clamp(maxAbsDutyCycle, 0.0, 1.0);
  }

  @Override
  public void setArmPosition(double rotations, int slot) {
    armControlMode = ArmControlMode.POSITION;
    requestedArmRotations = MathUtil.clamp(rotations, ARM_DEPLOYED_ROTATIONS, ARM_STOWED_ROTATIONS);
    requestedArmSlot = MathUtil.clamp(slot, ARM_FAST_SLOT, ARM_SLOW_SLOT);
  }

  @Override
  public void setArmVoltage(Voltage voltage) {
    armControlMode = ArmControlMode.VOLTAGE;
    requestedArmVoltage =
        MathUtil.clamp(voltage.in(Volts), -ARM_MAX_VOLTAGE.in(Volts), ARM_MAX_VOLTAGE.in(Volts));
  }

  @Override
  public void setArmDutyCycle(double output) {
    armControlMode = ArmControlMode.DUTY_CYCLE;
    requestedArmDutyCycle = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setArmTorqueCurrent(Current current) {
    armControlMode = ArmControlMode.TORQUE_CURRENT;
    requestedArmTorqueCurrentAmps =
        MathUtil.clamp(current.in(Amps), -ARM_STATOR_LIMIT.in(Amps), ARM_STATOR_LIMIT.in(Amps));
  }

  @Override
  public void setArmBrakeNeutral() {
    armControlMode = ArmControlMode.BRAKE;
  }

  private double calculateRollerVoltage(double rollerRotorSpeed) {
    return switch (rollerControlMode) {
      case VOLTAGE -> requestedRollerVoltage;
      case DUTY_CYCLE -> requestedRollerDutyCycle * ROLLER_MAX_VOLTAGE.in(Volts);
      case TORQUE_CURRENT -> MathUtil.clamp(
          ROLLER_SIM_MOTOR.getVoltage(
              ROLLER_SIM_MOTOR.getTorque(requestedRollerTorqueCurrentAmps), rollerRotorSpeed),
          -requestedRollerMaxDuty * 12.0,
          requestedRollerMaxDuty * 12.0);
    };
  }

  private double calculateArmVoltage(double armRotorSpeed) {
    return switch (armControlMode) {
      case POSITION -> {
        double currentArmRotations = Units.radiansToRotations(armSim.getAngleRads());
        double selectedKp = requestedArmSlot == ARM_FAST_SLOT ? ARM_FAST_KP : ARM_SLOW_KP;
        double feedbackVolts = selectedKp * (requestedArmRotations - currentArmRotations);
        double gravityVolts = ARM_KG * Math.cos(armSim.getAngleRads());
        yield feedbackVolts + gravityVolts;
      }
      case VOLTAGE -> requestedArmVoltage;
      case DUTY_CYCLE -> requestedArmDutyCycle * ARM_MAX_VOLTAGE.in(Volts);
      case TORQUE_CURRENT -> ARM_SIM_MOTOR.getVoltage(
          ARM_SIM_MOTOR.getTorque(requestedArmTorqueCurrentAmps), armRotorSpeed);
      case BRAKE -> 0.0;
    };
  }

  private static double limitVoltageForStatorCurrent(
      DCMotor motor, double rotorSpeedRadPerSec, double requestedVolts, double limitAmps) {
    double requestedCurrent = motor.getCurrent(rotorSpeedRadPerSec, requestedVolts);
    if (Math.abs(requestedCurrent) <= limitAmps) {
      return requestedVolts;
    }
    return motor.getVoltage(
        motor.getTorque(Math.copySign(limitAmps, requestedCurrent)), rotorSpeedRadPerSec);
  }
}
