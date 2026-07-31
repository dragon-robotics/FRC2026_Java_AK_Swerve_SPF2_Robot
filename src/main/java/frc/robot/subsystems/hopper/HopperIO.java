package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  class HopperIOInputs {
    public boolean leadConnected;
    public double leadPositionRotations;
    public double leadVelocityRpm;
    public double leadAppliedVolts;
    public double leadCurrentAmps;
    public double leadTempCelsius;
    public boolean followerConnected;
    public double followerPositionRotations;
    public double followerVelocityRpm;
    public double followerAppliedVolts;
    public double followerCurrentAmps;
    public double followerTempCelsius;
  }

  default void updateInputs(HopperIOInputs inputs) {}

  default void setVoltage(Voltage voltage) {}

  default void setDutyCycle(double output) {}

  default void setTorqueCurrent(Current current) {}
}
