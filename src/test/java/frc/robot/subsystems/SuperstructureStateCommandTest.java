package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure.ShootMode;
import frc.robot.subsystems.Superstructure.Superstate;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SuperstructureStateCommandTest {
  private SuperstructureTestHarness harness;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    harness = new SuperstructureTestHarness();
  }

  @AfterEach
  void tearDown() {
    harness.close();
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  @Test
  void defaultsAndLoggedStateUseSafeStartingConfiguration() {
    assertEquals(Superstate.DRIVE_STARTING_CONFIG, harness.superstructure.getCurrentState());
    assertEquals(ShootMode.DEFAULT_SHOOT_WITH_AIM, harness.superstructure.getShootMode());
    assertEquals(IntakeState.HOME, harness.intake.getDesiredState());
    assertEquals(HopperState.STOP, harness.hopper.getDesiredState());
    assertEquals(ShooterState.PREPFUEL, harness.shooter.getDesiredState());
  }

  @Test
  void coreStatesRequestExactMechanismStates() {
    assertState(Superstate.DRIVE, IntakeState.DEPLOYED, HopperState.STOP, ShooterState.PREPFUEL);
    assertState(Superstate.INTAKE, IntakeState.INTAKE, HopperState.STOP, ShooterState.PREPFUEL);
    assertState(
        Superstate.OUTTAKE,
        IntakeState.OUTTAKE,
        HopperState.INDEX_TO_INTAKE,
        ShooterState.PREPFUEL);
    assertState(
        Superstate.DRIVE_STARTING_CONFIG,
        IntakeState.HOME,
        HopperState.STOP,
        ShooterState.PREPFUEL);
  }

  @Test
  void mechanismDefaultsHoldOnlyTheLastIntakeRequest() {
    harness.run(harness.superstructure.setStateCmd(Superstate.OUTTAKE));

    CommandScheduler.getInstance().cancelAll();
    harness.runCycles(2);

    assertEquals(IntakeState.OUTTAKE, harness.intake.getDesiredState());
    assertEquals(HopperState.STOP, harness.hopper.getDesiredState());
    assertEquals(ShooterState.PREPFUEL, harness.shooter.getDesiredState());
  }

  @Test
  void nullConstructorDependenciesAndRequestedStatesAreRejected() {
    assertThrows(
        NullPointerException.class,
        () ->
            new Superstructure(
                null, harness.intake, harness.hopper, harness.shooter, harness.vision));
    assertThrows(
        NullPointerException.class,
        () ->
            new Superstructure(
                harness.drive, null, harness.hopper, harness.shooter, harness.vision));
    assertThrows(
        NullPointerException.class,
        () ->
            new Superstructure(
                harness.drive, harness.intake, null, harness.shooter, harness.vision));
    assertThrows(
        NullPointerException.class,
        () ->
            new Superstructure(
                harness.drive, harness.intake, harness.hopper, null, harness.vision));
    assertThrows(
        NullPointerException.class,
        () ->
            new Superstructure(
                harness.drive, harness.intake, harness.hopper, harness.shooter, null));
    assertThrows(
        NullPointerException.class,
        () ->
            new Superstructure(
                harness.drive,
                harness.intake,
                harness.hopper,
                harness.shooter,
                harness.vision,
                null));
    assertThrows(NullPointerException.class, () -> harness.superstructure.setStateCmd(null));
    assertThrows(NullPointerException.class, () -> harness.superstructure.intakeOverrideCmd(null));
    assertThrows(NullPointerException.class, () -> harness.superstructure.hopperOverrideCmd(null));
    assertThrows(NullPointerException.class, () -> harness.superstructure.shooterOverrideCmd(null));
  }

  private void assertState(
      Superstate state,
      IntakeState intakeState,
      HopperState hopperState,
      ShooterState shooterState) {
    harness.run(harness.superstructure.setStateCmd(state));
    assertEquals(state, harness.superstructure.getCurrentState());
    assertEquals(intakeState, harness.intake.getDesiredState());
    assertEquals(hopperState, harness.hopper.getDesiredState());
    assertEquals(shooterState, harness.shooter.getDesiredState());
  }
}
