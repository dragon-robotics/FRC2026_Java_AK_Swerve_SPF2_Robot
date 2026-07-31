package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

final class HopperConstants {
  static final int LEAD_MOTOR_ID = 17;
  static final int FOLLOWER_MOTOR_ID = 18;
  static final Current STATOR_CURRENT_LIMIT = Amps.of(80.0);
  static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40.0);
  static final Current SUPPLY_CURRENT_LOWER_LIMIT = Amps.of(20.0);
  static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.2);
  static final Voltage MAX_VOLTAGE = Volts.of(12.0);
  static final Voltage INDEX_TO_SHOOTER_VOLTAGE = Volts.of(12.0);
  static final Voltage INDEX_TO_INTAKE_VOLTAGE = Volts.of(-12.0);
  static final Voltage STOP_VOLTAGE = Volts.zero();
  static final Time OPEN_LOOP_RAMP = Seconds.of(0.5);
  static final double MAX_ABS_DUTY_CYCLE = 1.0;
  static final Current TORQUE_DEADBAND = Amps.of(1.0);
  static final double CONTROL_UPDATE_HZ = 100.0;
  static final double MECHANISM_STATUS_HZ = 50.0;
  static final double SLOW_STATUS_HZ = 4.0;
  static final DCMotor SIM_MOTOR = DCMotor.getKrakenX60Foc(1);
  static final double SIM_GEARING = 1.0;
  static final double SIM_MOI_KG_METERS_SQUARED = 0.004;

  static TalonFXConfiguration createLeadConfig() {
    return createMotorConfig(InvertedValue.CounterClockwise_Positive);
  }

  static TalonFXConfiguration createFollowerConfig() {
    return createMotorConfig(InvertedValue.Clockwise_Positive);
  }

  private static TalonFXConfiguration createMotorConfig(InvertedValue inversion) {
    return new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
                .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_TIME))
        .withVoltage(
            new VoltageConfigs()
                .withPeakForwardVoltage(MAX_VOLTAGE)
                .withPeakReverseVoltage(MAX_VOLTAGE.unaryMinus()))
        .withOpenLoopRamps(
            new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(OPEN_LOOP_RAMP)
                .withTorqueOpenLoopRampPeriod(OPEN_LOOP_RAMP)
                .withVoltageOpenLoopRampPeriod(OPEN_LOOP_RAMP))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(inversion));
  }

  private HopperConstants() {}
}
