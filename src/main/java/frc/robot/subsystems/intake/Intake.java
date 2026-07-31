package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public enum IntakeState {
    HOME,
    INTAKE,
    OUTTAKE,
    DEPLOYED,
    DEPLOYING,
    STOWING,
    JUICER
  }

  public enum JuicerPhase {
    PRE_JUICE,
    SQUEEZE
  }

  private final IntakeIO io;
  private final BooleanSupplier autonomousSupplier;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeState desiredState = IntakeState.HOME;
  private IntakeState currentState = IntakeState.STOWING;
  private IntakeState lastReconciledDesiredState = IntakeState.HOME;
  private IntakeState lastCommandedState;
  private JuicerPhase juicerPhase = JuicerPhase.PRE_JUICE;
  private JuicerPhase lastJuicerPhase;
  private Boolean lastDeployingAutonomousEnabled;

  public Intake(IntakeIO io) {
    this(io, DriverStation::isAutonomousEnabled);
  }

  Intake(IntakeIO io, BooleanSupplier autonomousSupplier) {
    this.io = Objects.requireNonNull(io);
    this.autonomousSupplier = Objects.requireNonNull(autonomousSupplier);
  }

  public IntakeState getDesiredState() {
    return desiredState;
  }

  public IntakeState getCurrentState() {
    return currentState;
  }

  public JuicerPhase getJuicerPhase() {
    return juicerPhase;
  }

  public boolean isArmAtDeployed() {
    return isArmAt(ARM_DEPLOYED_ROTATIONS);
  }

  public boolean isArmAtStowed() {
    return isArmAt(ARM_STOWED_ROTATIONS);
  }

  public boolean isArmAtPreJuice() {
    return isArmAt(ARM_PRE_JUICE_ROTATIONS);
  }

  public boolean isArmAtSqueeze() {
    return isArmAt(ARM_SQUEEZE_ROTATIONS);
  }

  public void setDesiredState(IntakeState state) {
    Objects.requireNonNull(state);
    if (state == IntakeState.DEPLOYING || state == IntakeState.STOWING) {
      throw new IllegalArgumentException("Transition states cannot be desired");
    }
    desiredState = state;
  }

  public void runRollerVoltage(Voltage voltage) {
    double requestedVolts = requireFinite(voltage.in(Volts), "roller voltage");
    io.setRollerVoltage(
        Volts.of(
            MathUtil.clamp(
                requestedVolts, -ROLLER_MAX_VOLTAGE.in(Volts), ROLLER_MAX_VOLTAGE.in(Volts))));
  }

  public void runRollerPercentage(double percentage) {
    io.setRollerDutyCycle(
        MathUtil.clamp(requireFinite(percentage, "roller percentage"), -1.0, 1.0));
  }

  public void runRollerTorqueCurrentFOC(Current current, double maxAbsDutyCycle) {
    double requestedCurrentAmps = requireFinite(current.in(Amps), "roller torque current");
    double requestedMaxDuty = requireFinite(maxAbsDutyCycle, "roller torque max duty");
    io.setRollerTorqueCurrent(
        Amps.of(
            MathUtil.clamp(
                requestedCurrentAmps, -ROLLER_STATOR_LIMIT.in(Amps), ROLLER_STATOR_LIMIT.in(Amps))),
        MathUtil.clamp(requestedMaxDuty, 0.0, 1.0));
  }

  public void runArmPosition(double rotations, int slot) {
    io.setArmPosition(
        MathUtil.clamp(
            requireFinite(rotations, "arm position"), ARM_DEPLOYED_ROTATIONS, ARM_STOWED_ROTATIONS),
        MathUtil.clamp(slot, ARM_FAST_SLOT, ARM_SLOW_SLOT));
  }

  public void runArmVoltage(Voltage voltage) {
    double requestedVolts = requireFinite(voltage.in(Volts), "arm voltage");
    io.setArmVoltage(
        Volts.of(
            MathUtil.clamp(requestedVolts, -ARM_MAX_VOLTAGE.in(Volts), ARM_MAX_VOLTAGE.in(Volts))));
  }

  public void runArmPercentage(double percentage) {
    io.setArmDutyCycle(MathUtil.clamp(requireFinite(percentage, "arm percentage"), -1.0, 1.0));
  }

  public void runArmTorqueCurrentFOC(Current current) {
    double requestedCurrentAmps = requireFinite(current.in(Amps), "arm torque current");
    io.setArmTorqueCurrent(
        Amps.of(
            MathUtil.clamp(
                requestedCurrentAmps, -ARM_STATOR_LIMIT.in(Amps), ARM_STATOR_LIMIT.in(Amps))));
  }

  private static double requireFinite(double value, String controlName) {
    if (!Double.isFinite(value)) {
      throw new IllegalArgumentException(controlName + " must be finite");
    }
    return value;
  }

  public void brakeArm() {
    io.setArmBrakeNeutral();
  }

  public void stopRoller() {
    runRollerVoltage(Volts.zero());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    reconcileDesiredState();

    switch (currentState) {
      case HOME -> handleHomeState();
      case INTAKE -> handleIntakeState();
      case OUTTAKE -> handleOuttakeState();
      case DEPLOYED -> handleDeployedState();
      case DEPLOYING -> handleDeployingState();
      case STOWING -> handleStowingState();
      case JUICER -> handleJuicerState();
    }
    Logger.recordOutput("Intake/DesiredState", desiredState.name());
    Logger.recordOutput("Intake/CurrentState", currentState.name());
    Logger.recordOutput("Intake/JuicerPhase", juicerPhase.name());
    Logger.recordOutput("Intake/MeasuredArmRotations", inputs.armPositionRotations);
    Logger.recordOutput("Intake/ArmAtDeployed", isArmAtDeployed());
    Logger.recordOutput("Intake/ArmAtStowed", isArmAtStowed());
    Logger.recordOutput("Intake/ArmAtPreJuice", isArmAtPreJuice());
  }

  private boolean isArmAt(double targetRotations) {
    return Math.abs(targetRotations - inputs.armPositionRotations)
        <= ARM_POSITION_TOLERANCE + Math.ulp(ARM_POSITION_TOLERANCE) * 8.0;
  }

  private void reconcileDesiredState() {
    if (desiredState == lastReconciledDesiredState) {
      return;
    }
    lastReconciledDesiredState = desiredState;
    lastCommandedState = null;
    switch (desiredState) {
      case HOME -> currentState = IntakeState.STOWING;
      case INTAKE, OUTTAKE, DEPLOYED -> {
        currentState = IntakeState.DEPLOYING;
        lastDeployingAutonomousEnabled = null;
      }
      case JUICER -> {
        currentState = IntakeState.JUICER;
        juicerPhase = JuicerPhase.PRE_JUICE;
        lastJuicerPhase = null;
      }
      case DEPLOYING, STOWING -> throw new IllegalStateException("Desired state was validated");
    }
  }

  private boolean isStateEntry() {
    return currentState != lastCommandedState;
  }

  private void markStateEntryHandled() {
    lastCommandedState = currentState;
  }

  private void deployArm() {
    runArmPosition(ARM_DEPLOYED_ROTATIONS, ARM_FAST_SLOT);
  }

  private void stowArm() {
    runArmPosition(ARM_STOWED_ROTATIONS, ARM_SLOW_SLOT);
  }

  private void handleHomeState() {}

  private void handleDeployingState() {
    if (isArmAtDeployed()) {
      enterDeployedTargetState();
      return;
    }
    if (isStateEntry()) {
      deployArm();
      markStateEntryHandled();
    }
    boolean autonomousEnabled = autonomousSupplier.getAsBoolean();
    if (lastDeployingAutonomousEnabled == null
        || autonomousEnabled != lastDeployingAutonomousEnabled) {
      runRollerVoltage(autonomousEnabled ? AUTONOMOUS_DEPLOY_ROLLER_VOLTAGE : Volts.zero());
      lastDeployingAutonomousEnabled = autonomousEnabled;
    }
  }

  private void enterDeployedTargetState() {
    IntakeState nextState =
        switch (desiredState) {
          case INTAKE -> IntakeState.INTAKE;
          case OUTTAKE -> IntakeState.OUTTAKE;
          default -> IntakeState.DEPLOYED;
        };
    switch (nextState) {
      case INTAKE -> commandIntakeOutputs();
      case OUTTAKE -> commandOuttakeOutputs();
      case DEPLOYED -> commandDeployedOutputs();
      default -> throw new IllegalStateException("Unexpected deployed target " + nextState);
    }
    currentState = nextState;
    lastCommandedState = nextState;
  }

  private void handleStowingState() {
    if (isStateEntry()) {
      stowArm();
      stopRoller();
      markStateEntryHandled();
    }
    if (isArmAtStowed()) {
      currentState = IntakeState.HOME;
      lastCommandedState = IntakeState.HOME;
    }
  }

  private void commandIntakeOutputs() {
    runArmTorqueCurrentFOC(ARM_INTAKE_TENSION_CURRENT);
    runRollerTorqueCurrentFOC(INTAKE_ROLLER_CURRENT, ROLLER_STATE_MAX_DUTY);
  }

  private void commandOuttakeOutputs() {
    brakeArm();
    runRollerTorqueCurrentFOC(OUTTAKE_ROLLER_CURRENT, ROLLER_STATE_MAX_DUTY);
  }

  private void commandDeployedOutputs() {
    brakeArm();
    stopRoller();
  }

  private void handleIntakeState() {
    if (!isStateEntry()) {
      return;
    }
    commandIntakeOutputs();
    markStateEntryHandled();
  }

  private void handleOuttakeState() {
    if (!isStateEntry()) {
      return;
    }
    commandOuttakeOutputs();
    markStateEntryHandled();
  }

  private void handleDeployedState() {
    if (!isStateEntry()) {
      return;
    }
    commandDeployedOutputs();
    markStateEntryHandled();
  }

  private void handleJuicerState() {
    if (lastJuicerPhase != juicerPhase) {
      if (juicerPhase == JuicerPhase.PRE_JUICE) {
        runRollerTorqueCurrentFOC(INTAKE_ROLLER_CURRENT, ROLLER_JUICER_MAX_DUTY);
        runArmPosition(ARM_PRE_JUICE_ROTATIONS, ARM_FAST_SLOT);
      } else {
        runArmPosition(ARM_SQUEEZE_ROTATIONS, ARM_SLOW_SLOT);
      }
      lastJuicerPhase = juicerPhase;
      lastCommandedState = IntakeState.JUICER;
      return;
    }
    if (juicerPhase == JuicerPhase.PRE_JUICE && isArmAtPreJuice()) {
      runArmPosition(ARM_SQUEEZE_ROTATIONS, ARM_SLOW_SLOT);
      juicerPhase = JuicerPhase.SQUEEZE;
      lastJuicerPhase = JuicerPhase.SQUEEZE;
    }
  }
}
