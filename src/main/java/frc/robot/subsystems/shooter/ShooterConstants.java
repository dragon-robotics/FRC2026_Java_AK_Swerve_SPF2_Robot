package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

final class ShooterConstants {
  static final int HOOD_MOTOR_ID = 13;
  static final int KICKER_MOTOR_ID = 14;
  static final int FLYWHEEL_LEAD_MOTOR_ID = 15;
  static final int FLYWHEEL_FOLLOWER_MOTOR_ID = 16;
  static final Current FLYWHEEL_STATOR_LIMIT = Amps.of(100.0);
  static final Current FLYWHEEL_FORWARD_TORQUE_LIMIT = Amps.of(120.0);
  static final Current FLYWHEEL_REVERSE_TORQUE_LIMIT = Amps.of(-40.0);
  static final Current KICKER_STATOR_LIMIT = Amps.of(80.0);
  static final Current HOOD_STATOR_LIMIT = Amps.of(30.0);
  static final Voltage FLYWHEEL_MAX_VOLTAGE = Volts.of(12.0);
  static final Voltage KICKER_MAX_VOLTAGE = Volts.of(12.0);
  static final Voltage HOOD_MAX_VOLTAGE = Volts.of(10.0);
  static final double DEFAULT_FLYWHEEL_RPM = 2500.0;
  static final double PREP_FLYWHEEL_RPM = 1200.0;
  static final double READY_BELOW_RPM = 120.0;
  static final double READY_ABOVE_RPM = 60.0;
  static final double STOPPED_TOLERANCE_RPM = 0.5;
  static final double DEFAULT_HOOD_ROTATIONS = 0.0;
  static final double HOOD_READY_TOLERANCE_ROTATIONS = 0.125;
  static final Voltage KICKER_PREP_VOLTAGE = Volts.of(6.0);
  static final Voltage KICKER_SHOOT_VOLTAGE = Volts.of(12.0);
  static final InterpolatingDoubleTreeMap FLYWHEEL_RPM_MAP = new InterpolatingDoubleTreeMap();
  static final InterpolatingDoubleTreeMap HOOD_ROTATIONS_MAP = new InterpolatingDoubleTreeMap();

  static {
    addSetpoint(5.0, 2400.0, 0.0);
    addSetpoint(6.0, 2475.0, 0.0);
    addSetpoint(7.0, 2525.0, 0.0);
    addSetpoint(8.0, 2675.0, 0.0);
    addSetpoint(9.0, 2750.0, 0.0);
    addSetpoint(10.0, 2850.0, 0.75);
    addSetpoint(11.0, 2900.0, 0.75);
    addSetpoint(12.0, 3000.0, 1.25);
  }

  record ShooterSetpoint(double flywheelRpm, double hoodRotations) {}

  static ShooterSetpoint getSetpointForDistance(double distanceMeters) {
    return new ShooterSetpoint(
        FLYWHEEL_RPM_MAP.get(distanceMeters), HOOD_ROTATIONS_MAP.get(distanceMeters));
  }

  private static void addSetpoint(double distanceFeet, double flywheelRpm, double hoodRotations) {
    double distanceMeters = Units.feetToMeters(distanceFeet);
    FLYWHEEL_RPM_MAP.put(distanceMeters, flywheelRpm);
    HOOD_ROTATIONS_MAP.put(distanceMeters, hoodRotations);
  }

  private ShooterConstants() {}
}
