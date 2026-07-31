package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public enum ShooterState {
    STOP,
    PREPFUEL,
    SHOOT,
    TRANSITION
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private ShooterState desiredState = ShooterState.STOP;
  private ShooterState currentState = ShooterState.TRANSITION;
  private double targetRpm = DEFAULT_FLYWHEEL_RPM;
  private double targetHoodRotations = DEFAULT_HOOD_ROTATIONS;

  public Shooter(ShooterIO io) {
    this.io = Objects.requireNonNull(io);
    io.resetHoodPosition(DEFAULT_HOOD_ROTATIONS);
  }

  public ShooterState getDesiredState() {
    return desiredState;
  }

  public ShooterState getCurrentState() {
    return currentState;
  }

  public double getTargetRpm() {
    return targetRpm;
  }

  public double getTargetHoodRotations() {
    return targetHoodRotations;
  }

  public double getMeasuredFlywheelRpm() {
    return inputs.flywheelLeadVelocityRpm;
  }

  public boolean isFlywheelReady() {
    return isFlywheelReadyFor(targetRpm);
  }

  public boolean isHoodReady() {
    return Math.abs(targetHoodRotations - inputs.hoodPositionRotations)
        <= HOOD_READY_TOLERANCE_ROTATIONS;
  }

  public void setDesiredState(ShooterState state) {
    Objects.requireNonNull(state);
    if (state == ShooterState.TRANSITION)
      throw new IllegalArgumentException("TRANSITION cannot be desired");
    desiredState = state;
    if (state != currentState) currentState = ShooterState.TRANSITION;
  }

  public void setSetpoint(double flywheelRpm, double hoodRotations) {
    boolean changed = targetRpm != flywheelRpm || targetHoodRotations != hoodRotations;
    targetRpm = flywheelRpm;
    targetHoodRotations = hoodRotations;
    if (changed && desiredState == ShooterState.SHOOT) currentState = ShooterState.TRANSITION;
  }

  public void setSetpointForDistance(double distanceMeters) {
    ShooterSetpoint setpoint = ShooterConstants.getSetpointForDistance(distanceMeters);
    setSetpoint(setpoint.flywheelRpm(), setpoint.hoodRotations());
  }

  public void runFlywheelVelocity(double rpm) {
    io.setFlywheelVelocity(rpm);
  }

  public void runFlywheelVoltage(Voltage voltage) {
    io.setFlywheelVoltage(clampVoltage(voltage, FLYWHEEL_MAX_VOLTAGE));
  }

  public void runFlywheelPercentage(double percentage) {
    io.setFlywheelDutyCycle(MathUtil.clamp(percentage, -1.0, 1.0));
  }

  public void runFlywheelTorqueCurrentFOC(Current current) {
    io.setFlywheelTorqueCurrent(
        Amps.of(
            MathUtil.clamp(
                current.in(Amps),
                FLYWHEEL_REVERSE_TORQUE_LIMIT.in(Amps),
                FLYWHEEL_STATOR_LIMIT.in(Amps))));
  }

  public void runKickerVoltage(Voltage voltage) {
    io.setKickerVoltage(clampVoltage(voltage, KICKER_MAX_VOLTAGE));
  }

  public void runKickerPercentage(double percentage) {
    io.setKickerDutyCycle(MathUtil.clamp(percentage, -1.0, 1.0));
  }

  public void runKickerTorqueCurrentFOC(Current current) {
    io.setKickerTorqueCurrent(
        Amps.of(
            MathUtil.clamp(
                current.in(Amps), -KICKER_STATOR_LIMIT.in(Amps), KICKER_STATOR_LIMIT.in(Amps))));
  }

  public void runHoodPosition(double rotations) {
    io.setHoodPosition(rotations);
  }

  public void runHoodVoltage(Voltage voltage) {
    io.setHoodVoltage(clampVoltage(voltage, HOOD_MAX_VOLTAGE));
  }

  public void runHoodPercentage(double percentage) {
    io.setHoodDutyCycle(MathUtil.clamp(percentage, -1.0, 1.0));
  }

  public void runHoodTorqueCurrentFOC(Current current) {
    io.setHoodTorqueCurrent(
        Amps.of(
            MathUtil.clamp(
                current.in(Amps), -HOOD_STATOR_LIMIT.in(Amps), HOOD_STATOR_LIMIT.in(Amps))));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    handleState();
    Logger.recordOutput("Shooter/CurrentState", currentState.name());
    Logger.recordOutput("Shooter/DesiredState", desiredState.name());
    Logger.recordOutput("Shooter/TargetRPM", targetRpm);
    Logger.recordOutput("Shooter/TargetHoodRotations", targetHoodRotations);
    Logger.recordOutput("Shooter/FlywheelReady", isFlywheelReady());
    Logger.recordOutput("Shooter/HoodReady", isHoodReady());
  }

  private void handleState() {
    switch (currentState) {
      case STOP -> {}
      case PREPFUEL -> commandPrepFuel();
      case SHOOT -> commandShoot();
      case TRANSITION -> {
        switch (desiredState) {
          case STOP -> transitionToStop();
          case PREPFUEL -> transitionToPrepFuel();
          case SHOOT -> transitionToShoot();
          case TRANSITION -> throw new IllegalStateException("TRANSITION cannot be desired");
        }
      }
    }
  }

  private void transitionToStop() {
    runFlywheelVoltage(Volts.zero());
    runKickerVoltage(Volts.zero());
    runHoodPosition(DEFAULT_HOOD_ROTATIONS);
    if (Math.abs(inputs.flywheelLeadVelocityRpm) < STOPPED_TOLERANCE_RPM)
      currentState = ShooterState.STOP;
  }

  private void transitionToPrepFuel() {
    commandPrepFuel();
    if (isFlywheelReadyFor(PREP_FLYWHEEL_RPM)) currentState = ShooterState.PREPFUEL;
  }

  private void transitionToShoot() {
    runFlywheelVelocity(targetRpm);
    runHoodPosition(targetHoodRotations);
    runKickerVoltage(KICKER_PREP_VOLTAGE);
    if (isFlywheelReady() && isHoodReady()) currentState = ShooterState.SHOOT;
  }

  private void commandPrepFuel() {
    runFlywheelVelocity(PREP_FLYWHEEL_RPM);
    runKickerVoltage(KICKER_PREP_VOLTAGE);
    runHoodPosition(DEFAULT_HOOD_ROTATIONS);
  }

  private void commandShoot() {
    runFlywheelVelocity(targetRpm);
    runHoodPosition(targetHoodRotations);
    runKickerVoltage(KICKER_SHOOT_VOLTAGE);
  }

  private boolean isFlywheelReadyFor(double requestedRpm) {
    double actualRpm = inputs.flywheelLeadVelocityRpm;
    return actualRpm >= requestedRpm - READY_BELOW_RPM
        && actualRpm <= requestedRpm + READY_ABOVE_RPM;
  }

  private static Voltage clampVoltage(Voltage voltage, Voltage maximum) {
    return Volts.of(MathUtil.clamp(voltage.in(Volts), -maximum.in(Volts), maximum.in(Volts)));
  }
}
