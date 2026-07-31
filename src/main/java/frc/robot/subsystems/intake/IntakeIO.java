package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  final class NoOp implements IntakeIO {}

  @AutoLog
  class IntakeIOInputs {
    public boolean rollerLeadConnected;
    public double rollerLeadPositionRotations;
    public double rollerLeadVelocityRpm;
    public double rollerLeadAppliedVolts;
    public double rollerLeadCurrentAmps;
    public double rollerLeadTempCelsius;
    public boolean rollerFollowerConnected;
    public double rollerFollowerPositionRotations;
    public double rollerFollowerVelocityRpm;
    public double rollerFollowerAppliedVolts;
    public double rollerFollowerCurrentAmps;
    public double rollerFollowerTempCelsius;
    public boolean armConnected;
    public double armPositionRotations;
    public double armVelocityRpm;
    public double armAppliedVolts;
    public double armCurrentAmps;
    public double armTempCelsius;
    public boolean armCancoderConnected;
    public double armCancoderPositionRotations;
    public double armCancoderAbsolutePositionRotations;
    public double armCancoderVelocityRpm;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setRollerVoltage(Voltage voltage) {}

  default void setRollerDutyCycle(double output) {}

  default void setRollerTorqueCurrent(Current current, double maxAbsDutyCycle) {}

  default void setArmPosition(double rotations, int slot) {}

  default void setArmVoltage(Voltage voltage) {}

  default void setArmDutyCycle(double output) {}

  default void setArmTorqueCurrent(Current current) {}

  default void setArmBrakeNeutral() {}
}
