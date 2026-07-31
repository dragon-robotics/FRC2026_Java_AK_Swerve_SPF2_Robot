package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hopper.HopperConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  public enum HopperState {
    STOP,
    INDEX_TO_SHOOTER,
    INDEX_TO_INTAKE
  }

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  private HopperState desiredState = HopperState.STOP;
  private HopperState currentState = HopperState.STOP;
  private HopperState lastCommandedState;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  public HopperState getDesiredState() {
    return desiredState;
  }

  public HopperState getCurrentState() {
    return currentState;
  }

  public void setDesiredState(HopperState state) {
    desiredState = Objects.requireNonNull(state);
  }

  public void runVoltage(Voltage voltage) {
    io.setVoltage(
        Volts.of(MathUtil.clamp(voltage.in(Volts), -MAX_VOLTAGE.in(Volts), MAX_VOLTAGE.in(Volts))));
  }

  public void runPercentage(double percentage) {
    io.setDutyCycle(MathUtil.clamp(percentage, -1.0, 1.0));
  }

  public void runTorqueCurrentFOC(Current current) {
    io.setTorqueCurrent(
        Amps.of(
            MathUtil.clamp(
                current.in(Amps), -STATOR_CURRENT_LIMIT.in(Amps), STATOR_CURRENT_LIMIT.in(Amps))));
  }

  public void indexToShooter() {
    runVoltage(INDEX_TO_SHOOTER_VOLTAGE);
  }

  public void indexToIntake() {
    runVoltage(INDEX_TO_INTAKE_VOLTAGE);
  }

  public void stop() {
    runVoltage(STOP_VOLTAGE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    if (desiredState != lastCommandedState) {
      switch (desiredState) {
        case STOP -> stop();
        case INDEX_TO_SHOOTER -> indexToShooter();
        case INDEX_TO_INTAKE -> indexToIntake();
      }
      currentState = desiredState;
      lastCommandedState = desiredState;
    }
    Logger.recordOutput("Hopper/CurrentState", currentState.name());
    Logger.recordOutput("Hopper/DesiredState", desiredState.name());
  }
}
