package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  final class NoOp implements ShooterIO {}

  @AutoLog
  class ShooterIOInputs {
    public boolean flywheelLeadConnected;
    public double flywheelLeadPositionRotations;
    public double flywheelLeadVelocityRpm;
    public double flywheelLeadAppliedVolts;
    public double flywheelLeadCurrentAmps;
    public double flywheelLeadTempCelsius;
    public boolean flywheelFollowerConnected;
    public double flywheelFollowerPositionRotations;
    public double flywheelFollowerVelocityRpm;
    public double flywheelFollowerAppliedVolts;
    public double flywheelFollowerCurrentAmps;
    public double flywheelFollowerTempCelsius;
    public boolean kickerConnected;
    public double kickerPositionRotations;
    public double kickerVelocityRpm;
    public double kickerAppliedVolts;
    public double kickerCurrentAmps;
    public double kickerTempCelsius;
    public boolean hoodConnected;
    public double hoodPositionRotations;
    public double hoodVelocityRpm;
    public double hoodAppliedVolts;
    public double hoodCurrentAmps;
    public double hoodTempCelsius;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setFlywheelVelocity(double rpm) {}

  default void setFlywheelVoltage(Voltage voltage) {}

  default void setFlywheelDutyCycle(double output) {}

  default void setFlywheelTorqueCurrent(Current current) {}

  default void setKickerVoltage(Voltage voltage) {}

  default void setKickerDutyCycle(double output) {}

  default void setKickerTorqueCurrent(Current current) {}

  default void setHoodPosition(double rotations) {}

  default void setHoodVoltage(Voltage voltage) {}

  default void setHoodDutyCycle(double output) {}

  default void setHoodTorqueCurrent(Current current) {}

  default void resetHoodPosition(double rotations) {}
}
