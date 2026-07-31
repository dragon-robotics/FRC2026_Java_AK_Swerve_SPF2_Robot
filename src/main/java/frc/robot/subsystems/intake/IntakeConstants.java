package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

final class IntakeConstants {
  static final int ROLLER_LEAD_ID = 21;
  static final int ROLLER_FOLLOWER_ID = 20;
  static final int ARM_MOTOR_ID = 10;
  static final int ARM_CANCODER_ID = 0;
  static final int ARM_FAST_SLOT = 0;
  static final int ARM_SLOW_SLOT = 1;
  static final double ARM_DEPLOYED_ROTATIONS = 0.0;
  static final double ARM_PRE_JUICE_ROTATIONS = 0.15;
  static final double ARM_SQUEEZE_ROTATIONS = 0.25;
  static final double ARM_STOWED_ROTATIONS = 0.37;
  static final double ARM_POSITION_TOLERANCE = 0.025;
  static final Current ROLLER_STATOR_LIMIT = Amps.of(80.0);
  static final Current ARM_STATOR_LIMIT = Amps.of(50.0);
  static final Current INTAKE_ROLLER_CURRENT = Amps.of(80.0);
  static final Current OUTTAKE_ROLLER_CURRENT = Amps.of(-80.0);
  static final Current ARM_INTAKE_TENSION_CURRENT = Amps.of(-15.0);
  static final double ROLLER_STATE_MAX_DUTY = 0.80;
  static final double ROLLER_JUICER_MAX_DUTY = 0.50;
  static final Voltage ROLLER_MAX_VOLTAGE = Volts.of(12.0);
  static final Voltage ARM_MAX_VOLTAGE = Volts.of(10.0);
  static final Voltage AUTONOMOUS_DEPLOY_ROLLER_VOLTAGE = Volts.of(-6.0);
  static final double CONTROL_HZ = 100.0;
  static final double MECHANISM_HZ = 50.0;
  static final double SLOW_HZ = 4.0;

  private IntakeConstants() {}
}
