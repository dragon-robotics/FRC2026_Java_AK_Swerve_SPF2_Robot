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
  private static final int BATTERY_SOLVER_ITERATIONS = 50;
  private static final double BATTERY_SOLVER_TOLERANCE_VOLTS = 1e-12;
  private static final double MINIMUM_LOADED_BUS_VOLTS =
      BatterySim.calculateDefaultBatteryLoadedVoltage(
          ROLLER_STATOR_LIMIT.in(Amps), ROLLER_STATOR_LIMIT.in(Amps), ARM_STATOR_LIMIT.in(Amps));
  private static final double MAXIMUM_LOADED_BUS_VOLTS =
      BatterySim.calculateDefaultBatteryLoadedVoltage(
          -ROLLER_STATOR_LIMIT.in(Amps), -ROLLER_STATOR_LIMIT.in(Amps), -ARM_STATOR_LIMIT.in(Amps));

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

    SolvedCycle solvedCycle = solveLoadedCycle(leadRotorSpeed, followerRotorSpeed, armRotorSpeed);
    AppliedCycle appliedCycle = solvedCycle.appliedCycle();
    RoboRioSim.setVInVoltage(solvedCycle.loadedBusVolts());

    rollerLeadSim.setInputVoltage(appliedCycle.rollerLeadVolts());
    rollerFollowerSim.setInputVoltage(appliedCycle.rollerFollowerVolts());
    armSim.setInputVoltage(appliedCycle.armVolts());
    rollerLeadSim.update(LOOP_PERIOD_SECONDS);
    rollerFollowerSim.update(LOOP_PERIOD_SECONDS);
    armSim.update(LOOP_PERIOD_SECONDS);

    inputs.rollerLeadConnected = true;
    inputs.rollerLeadPositionRotations = rollerLeadSim.getAngularPositionRotations();
    inputs.rollerLeadVelocityRpm = rollerLeadSim.getAngularVelocityRPM();
    inputs.rollerLeadAppliedVolts = rollerLeadSim.getInputVoltage();
    inputs.rollerLeadCurrentAmps =
        Math.min(
            ROLLER_STATOR_LIMIT.in(Amps), Math.abs(appliedCycle.rollerLeadStatorCurrentAmps()));
    inputs.rollerLeadTempCelsius = 0.0;

    inputs.rollerFollowerConnected = true;
    inputs.rollerFollowerPositionRotations = rollerFollowerSim.getAngularPositionRotations();
    inputs.rollerFollowerVelocityRpm = rollerFollowerSim.getAngularVelocityRPM();
    inputs.rollerFollowerAppliedVolts = rollerFollowerSim.getInputVoltage();
    inputs.rollerFollowerCurrentAmps =
        Math.min(
            ROLLER_STATOR_LIMIT.in(Amps), Math.abs(appliedCycle.rollerFollowerStatorCurrentAmps()));
    inputs.rollerFollowerTempCelsius = 0.0;

    double armPositionRotations = Units.radiansToRotations(armSim.getAngleRads());
    double armVelocityRpm =
        Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec());
    inputs.armConnected = true;
    inputs.armPositionRotations = armPositionRotations;
    inputs.armVelocityRpm = armVelocityRpm;
    inputs.armAppliedVolts = armSim.getInput(0);
    inputs.armCurrentAmps =
        Math.min(ARM_STATOR_LIMIT.in(Amps), Math.abs(appliedCycle.armStatorCurrentAmps()));
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

  private SolvedCycle solveLoadedCycle(
      double leadRotorSpeed, double followerRotorSpeed, double armRotorSpeed) {
    double lowerBusVolts = MINIMUM_LOADED_BUS_VOLTS;
    double upperBusVolts = MAXIMUM_LOADED_BUS_VOLTS;
    double loadedBusVolts = upperBusVolts;
    AppliedCycle appliedCycle =
        calculateAppliedCycle(leadRotorSpeed, followerRotorSpeed, armRotorSpeed, loadedBusVolts);
    for (int iteration = 0; iteration < BATTERY_SOLVER_ITERATIONS; iteration++) {
      loadedBusVolts = (lowerBusVolts + upperBusVolts) / 2.0;
      appliedCycle =
          calculateAppliedCycle(leadRotorSpeed, followerRotorSpeed, armRotorSpeed, loadedBusVolts);
      double nextLoadedBusVolts = calculateLoadedBusVoltage(appliedCycle);
      if (Math.abs(nextLoadedBusVolts - loadedBusVolts) <= BATTERY_SOLVER_TOLERANCE_VOLTS) {
        break;
      }
      if (nextLoadedBusVolts > loadedBusVolts) {
        lowerBusVolts = loadedBusVolts;
      } else {
        upperBusVolts = loadedBusVolts;
      }
    }
    double residualVolts = calculateLoadedBusVoltage(appliedCycle) - loadedBusVolts;
    if (!Double.isFinite(residualVolts)
        || Math.abs(residualVolts) > BATTERY_SOLVER_TOLERANCE_VOLTS) {
      throw new IllegalStateException(
          "Intake battery solver failed to converge at "
              + loadedBusVolts
              + " V with residual "
              + residualVolts
              + " V");
    }
    return new SolvedCycle(loadedBusVolts, appliedCycle);
  }

  private AppliedCycle calculateAppliedCycle(
      double leadRotorSpeed,
      double followerRotorSpeed,
      double armRotorSpeed,
      double loadedBusVolts) {
    double rollerVoltageLimit = Math.min(loadedBusVolts, ROLLER_MAX_VOLTAGE.in(Volts));
    double requestedLeadVolts = calculateRollerVoltage(leadRotorSpeed, loadedBusVolts);
    double leadVolts =
        applyVoltageAndCurrentLimits(
            ROLLER_SIM_MOTOR,
            leadRotorSpeed,
            requestedLeadVolts,
            rollerVoltageLimit,
            ROLLER_STATOR_LIMIT.in(Amps));
    double followerVolts =
        applyVoltageAndCurrentLimits(
            ROLLER_SIM_MOTOR,
            followerRotorSpeed,
            -requestedLeadVolts,
            rollerVoltageLimit,
            ROLLER_STATOR_LIMIT.in(Amps));
    double armVoltageLimit = Math.min(loadedBusVolts, ARM_MAX_VOLTAGE.in(Volts));
    double armVolts =
        applyVoltageAndCurrentLimits(
            ARM_SIM_MOTOR,
            armRotorSpeed,
            calculateArmVoltage(armRotorSpeed, loadedBusVolts),
            armVoltageLimit,
            ARM_STATOR_LIMIT.in(Amps));
    return new AppliedCycle(
        leadVolts,
        followerVolts,
        armVolts,
        ROLLER_SIM_MOTOR.getCurrent(leadRotorSpeed, leadVolts),
        ROLLER_SIM_MOTOR.getCurrent(followerRotorSpeed, followerVolts),
        ARM_SIM_MOTOR.getCurrent(armRotorSpeed, armVolts));
  }

  private static double calculateLoadedBusVoltage(AppliedCycle appliedCycle) {
    return BatterySim.calculateDefaultBatteryLoadedVoltage(
        calculateBatteryCurrent(
            appliedCycle.rollerLeadStatorCurrentAmps(), appliedCycle.rollerLeadVolts()),
        calculateBatteryCurrent(
            appliedCycle.rollerFollowerStatorCurrentAmps(), appliedCycle.rollerFollowerVolts()),
        calculateBatteryCurrent(appliedCycle.armStatorCurrentAmps(), appliedCycle.armVolts()));
  }

  private static double calculateBatteryCurrent(double statorCurrentAmps, double appliedVolts) {
    return statorCurrentAmps * Math.signum(appliedVolts);
  }

  private double calculateRollerVoltage(double rollerRotorSpeed, double loadedBusVolts) {
    return switch (rollerControlMode) {
      case VOLTAGE -> requestedRollerVoltage;
      case DUTY_CYCLE -> requestedRollerDutyCycle * loadedBusVolts;
      case TORQUE_CURRENT -> MathUtil.clamp(
          ROLLER_SIM_MOTOR.getVoltage(
              ROLLER_SIM_MOTOR.getTorque(requestedRollerTorqueCurrentAmps), rollerRotorSpeed),
          -requestedRollerMaxDuty * loadedBusVolts,
          requestedRollerMaxDuty * loadedBusVolts);
    };
  }

  private double calculateArmVoltage(double armRotorSpeed, double loadedBusVolts) {
    return switch (armControlMode) {
      case POSITION -> {
        double currentArmRotations = Units.radiansToRotations(armSim.getAngleRads());
        double selectedKp = requestedArmSlot == ARM_FAST_SLOT ? ARM_FAST_KP : ARM_SLOW_KP;
        double feedbackVolts = selectedKp * (requestedArmRotations - currentArmRotations);
        double gravityVolts = ARM_KG * Math.cos(armSim.getAngleRads());
        yield feedbackVolts + gravityVolts;
      }
      case VOLTAGE -> requestedArmVoltage;
      case DUTY_CYCLE -> requestedArmDutyCycle * loadedBusVolts;
      case TORQUE_CURRENT -> ARM_SIM_MOTOR.getVoltage(
          ARM_SIM_MOTOR.getTorque(requestedArmTorqueCurrentAmps), armRotorSpeed);
      case BRAKE -> 0.0;
    };
  }

  private static double applyVoltageAndCurrentLimits(
      DCMotor motor,
      double rotorSpeedRadPerSec,
      double requestedVolts,
      double voltageLimit,
      double currentLimitAmps) {
    double voltageCappedRequest = MathUtil.clamp(requestedVolts, -voltageLimit, voltageLimit);
    double currentLimitedRequest =
        limitVoltageForStatorCurrent(
            motor, rotorSpeedRadPerSec, voltageCappedRequest, currentLimitAmps);
    return MathUtil.clamp(currentLimitedRequest, -voltageLimit, voltageLimit);
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

  private record AppliedCycle(
      double rollerLeadVolts,
      double rollerFollowerVolts,
      double armVolts,
      double rollerLeadStatorCurrentAmps,
      double rollerFollowerStatorCurrentAmps,
      double armStatorCurrentAmps) {}

  private record SolvedCycle(double loadedBusVolts, AppliedCycle appliedCycle) {}
}
