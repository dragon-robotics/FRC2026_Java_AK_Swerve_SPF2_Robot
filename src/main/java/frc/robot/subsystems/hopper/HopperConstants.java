package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

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

  private HopperConstants() {}
}
