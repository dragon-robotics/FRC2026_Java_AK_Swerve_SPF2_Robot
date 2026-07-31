package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.Intake.JuicerPhase;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class IntakeTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  private static final class RecordingIO implements IntakeIO {
    double measuredArmRotations;
    Double rollerVolts;
    Double rollerDutyCycle;
    Double rollerTorqueAmps;
    Double rollerMaxDuty;
    Double armPositionRotations;
    Integer armSlot;
    Double armVolts;
    Double armDutyCycle;
    Double armTorqueAmps;
    int armBrakeNeutralCalls;
    int armPositionCalls;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
      inputs.armPositionRotations = measuredArmRotations;
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
      rollerVolts = voltage.in(Volts);
    }

    @Override
    public void setRollerDutyCycle(double output) {
      rollerDutyCycle = output;
    }

    @Override
    public void setRollerTorqueCurrent(Current current, double maxAbsDutyCycle) {
      rollerTorqueAmps = current.in(Amps);
      rollerMaxDuty = maxAbsDutyCycle;
    }

    @Override
    public void setArmPosition(double rotations, int slot) {
      armPositionRotations = rotations;
      armSlot = slot;
      armPositionCalls++;
    }

    @Override
    public void setArmVoltage(Voltage voltage) {
      armVolts = voltage.in(Volts);
    }

    @Override
    public void setArmDutyCycle(double output) {
      armDutyCycle = output;
    }

    @Override
    public void setArmTorqueCurrent(Current current) {
      armTorqueAmps = current.in(Amps);
    }

    @Override
    public void setArmBrakeNeutral() {
      armBrakeNeutralCalls++;
    }
  }

  @Test
  void firstPeriodicExplicitlyCommandsSafeHome() {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.37;
    Intake intake = new Intake(io, () -> false);

    intake.periodic();

    assertEquals(0.0, io.rollerVolts, 1e-9);
    assertEquals(0.37, io.armPositionRotations, 1e-9);
    assertEquals(1, io.armSlot);
    assertEquals(IntakeState.HOME, intake.getCurrentState());
  }

  @Test
  void teleopDeployStopsRollerThenEntersBrakeNeutralDeployedState() {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.37;
    Intake intake = new Intake(io, () -> false);
    intake.periodic();
    intake.setDesiredState(IntakeState.DEPLOYED);
    io.measuredArmRotations = 0.20;

    intake.periodic();

    assertEquals(IntakeState.DEPLOYING, intake.getCurrentState());
    assertEquals(0.0, io.armPositionRotations, 1e-9);
    assertEquals(0, io.armSlot);
    assertEquals(0.0, io.rollerVolts, 1e-9);

    io.measuredArmRotations = 0.025;
    intake.periodic();

    assertEquals(IntakeState.DEPLOYED, intake.getCurrentState());
    assertEquals(1, io.armBrakeNeutralCalls);
    assertEquals(0.0, io.rollerVolts, 1e-9);
  }

  @Test
  void autonomousDeployRunsRollerOutwardAtSixVolts() {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.37;
    Intake intake = new Intake(io, () -> true);
    intake.periodic();
    intake.setDesiredState(IntakeState.INTAKE);

    intake.periodic();

    assertEquals(IntakeState.DEPLOYING, intake.getCurrentState());
    assertEquals(-6.0, io.rollerVolts, 1e-9);
  }

  @Test
  void disabledAutonomousDeploymentStopsRoller() {
    DriverStationSim.resetData();
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    try {
      RecordingIO io = new RecordingIO();
      io.measuredArmRotations = 0.37;
      Intake intake = new Intake(io);
      intake.periodic();
      intake.setDesiredState(IntakeState.INTAKE);

      intake.periodic();

      assertEquals(IntakeState.DEPLOYING, intake.getCurrentState());
      assertEquals(0.0, io.rollerVolts, 1e-9);
    } finally {
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    }
  }

  @Test
  void deploymentReadinessAppliesIntakeOutputsBeforePublishingIntakeState() {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.37;
    Intake intake = new Intake(io, () -> false);
    intake.periodic();
    intake.setDesiredState(IntakeState.INTAKE);
    intake.periodic();

    io.measuredArmRotations = 0.025;
    intake.periodic();

    assertEquals(IntakeState.INTAKE, intake.getCurrentState());
    assertEquals(80.0, io.rollerTorqueAmps, 1e-9);
    assertEquals(0.80, io.rollerMaxDuty, 1e-9);
    assertEquals(-15.0, io.armTorqueAmps, 1e-9);
  }

  @Test
  void desiredRequestDoesNotPublishTransitionBeforeItsOutputs() {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.37;
    Intake intake = new Intake(io, () -> false);
    intake.periodic();

    intake.setDesiredState(IntakeState.INTAKE);

    assertEquals(IntakeState.HOME, intake.getCurrentState());
    assertEquals(IntakeState.INTAKE, intake.getDesiredState());

    intake.periodic();
    assertEquals(IntakeState.DEPLOYING, intake.getCurrentState());
    assertEquals(0.0, io.armPositionRotations, 1e-9);
    assertEquals(0.0, io.rollerVolts, 1e-9);
  }

  @Test
  void outtakeUsesBrakeNeutralAndNegativeRollerTorque() {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.0;
    Intake intake = new Intake(io, () -> false);
    intake.periodic();
    intake.setDesiredState(IntakeState.OUTTAKE);

    intake.periodic();

    assertEquals(IntakeState.OUTTAKE, intake.getCurrentState());
    assertEquals(-80.0, io.rollerTorqueAmps, 1e-9);
    assertEquals(0.80, io.rollerMaxDuty, 1e-9);
    assertEquals(1, io.armBrakeNeutralCalls);
    assertNull(io.armTorqueAmps);
  }

  @Test
  void juicerUsesHalfDutyAndChangesToSlowSqueezeAtInclusiveBoundary() {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.0;
    Intake intake = new Intake(io, () -> false);
    intake.periodic();
    intake.setDesiredState(IntakeState.JUICER);

    intake.periodic();

    assertEquals(80.0, io.rollerTorqueAmps, 1e-9);
    assertEquals(0.50, io.rollerMaxDuty, 1e-9);
    assertEquals(0.15, io.armPositionRotations, 1e-9);
    assertEquals(0, io.armSlot);
    assertEquals(JuicerPhase.PRE_JUICE, intake.getJuicerPhase());

    io.measuredArmRotations = 0.125;
    intake.periodic();

    assertEquals(JuicerPhase.SQUEEZE, intake.getJuicerPhase());
    assertEquals(0.25, io.armPositionRotations, 1e-9);
    assertEquals(1, io.armSlot);
  }

  @Test
  void brakeNeutralStateDoesNotAutomaticallyRedeployAfterCollision() {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.0;
    Intake intake = new Intake(io, () -> false);
    intake.periodic();
    intake.setDesiredState(IntakeState.DEPLOYED);
    intake.periodic();
    int positionCallsBeforeCollision = io.armPositionCalls;

    io.measuredArmRotations = 0.10;
    intake.periodic();

    assertEquals(IntakeState.DEPLOYED, intake.getCurrentState());
    assertEquals(positionCallsBeforeCollision, io.armPositionCalls);
  }

  @Test
  void laterStateChangeRedeploysArmDisplacedByCollision() {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.0;
    Intake intake = new Intake(io, () -> false);
    intake.periodic();
    intake.setDesiredState(IntakeState.DEPLOYED);
    intake.periodic();
    io.measuredArmRotations = 0.10;
    int positionCallsBeforeNewRequest = io.armPositionCalls;

    intake.setDesiredState(IntakeState.OUTTAKE);
    intake.periodic();

    assertEquals(IntakeState.DEPLOYING, intake.getCurrentState());
    assertEquals(positionCallsBeforeNewRequest + 1, io.armPositionCalls);
    assertEquals(0.0, io.armPositionRotations, 1e-9);
    assertEquals(0, io.armSlot);
  }

  @Test
  void homeRequestStowsWithSlowSlotAndLeavesHoldRequestActive() {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.0;
    Intake intake = new Intake(io, () -> false);
    intake.periodic();
    intake.setDesiredState(IntakeState.DEPLOYED);
    intake.periodic();

    intake.setDesiredState(IntakeState.HOME);
    intake.periodic();
    int positionCallsBeforeReady = io.armPositionCalls;
    assertEquals(IntakeState.STOWING, intake.getCurrentState());
    assertEquals(0.37, io.armPositionRotations, 1e-9);
    assertEquals(1, io.armSlot);
    assertEquals(0.0, io.rollerVolts, 1e-9);

    io.measuredArmRotations = 0.345;
    intake.periodic();
    assertEquals(IntakeState.HOME, intake.getCurrentState());
    assertEquals(positionCallsBeforeReady, io.armPositionCalls);
    assertEquals(0.37, io.armPositionRotations, 1e-9);
  }

  @Test
  void repeatedRequestForActiveStateDoesNotResendOutputs() {
    RecordingIO io = new RecordingIO();
    io.measuredArmRotations = 0.0;
    Intake intake = new Intake(io, () -> false);
    intake.periodic();
    intake.setDesiredState(IntakeState.DEPLOYED);
    intake.periodic();
    int brakeCalls = io.armBrakeNeutralCalls;

    intake.setDesiredState(IntakeState.DEPLOYED);
    intake.periodic();

    assertEquals(brakeCalls, io.armBrakeNeutralCalls);
  }

  @Test
  void readinessUsesInclusivePointZeroTwoFiveTolerance() {
    RecordingIO io = new RecordingIO();
    Intake intake = new Intake(io, () -> false);

    io.measuredArmRotations = 0.025;
    intake.periodic();
    assertTrue(intake.isArmAtDeployed());
    io.measuredArmRotations = 0.02501;
    intake.periodic();
    assertFalse(intake.isArmAtDeployed());
    io.measuredArmRotations = 0.345;
    intake.periodic();
    assertTrue(intake.isArmAtStowed());
    io.measuredArmRotations = 0.12499;
    intake.periodic();
    assertFalse(intake.isArmAtPreJuice());
  }

  @Test
  void invalidDesiredStatesAreRejectedWithoutCorruptingDesiredState() {
    Intake intake = new Intake(new RecordingIO(), () -> false);

    assertThrows(NullPointerException.class, () -> intake.setDesiredState(null));
    assertThrows(
        IllegalArgumentException.class, () -> intake.setDesiredState(IntakeState.DEPLOYING));
    assertThrows(IllegalArgumentException.class, () -> intake.setDesiredState(IntakeState.STOWING));
    assertEquals(IntakeState.HOME, intake.getDesiredState());
  }

  @Test
  void directControlsClampAtSubsystemBoundary() {
    RecordingIO io = new RecordingIO();
    Intake intake = new Intake(io, () -> false);

    intake.runRollerVoltage(Volts.of(20.0));
    intake.runRollerPercentage(-2.0);
    intake.runRollerTorqueCurrentFOC(Amps.of(100.0), 1.25);
    intake.runArmVoltage(Volts.of(-20.0));
    intake.runArmPercentage(2.0);
    intake.runArmTorqueCurrentFOC(Amps.of(-75.0));
    intake.runArmPosition(0.80, 9);
    intake.brakeArm();

    assertEquals(12.0, io.rollerVolts, 1e-9);
    assertEquals(-1.0, io.rollerDutyCycle, 1e-9);
    assertEquals(80.0, io.rollerTorqueAmps, 1e-9);
    assertEquals(1.0, io.rollerMaxDuty, 1e-9);
    assertEquals(-10.0, io.armVolts, 1e-9);
    assertEquals(1.0, io.armDutyCycle, 1e-9);
    assertEquals(-50.0, io.armTorqueAmps, 1e-9);
    assertEquals(0.37, io.armPositionRotations, 1e-9);
    assertEquals(1, io.armSlot);
    assertEquals(1, io.armBrakeNeutralCalls);
  }
}
