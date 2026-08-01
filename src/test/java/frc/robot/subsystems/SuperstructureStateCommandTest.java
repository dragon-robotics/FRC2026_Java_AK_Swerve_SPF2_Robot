package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.ShootMode;
import frc.robot.subsystems.Superstructure.Superstate;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
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
  void cleanupAfterFailedConstructionRemovesAllSchedulerRegistrations() {
    AtomicInteger periodicCalls = new AtomicInteger();
    new SubsystemBase() {
      @Override
      public void periodic() {
        periodicCalls.incrementAndGet();
      }
    };
    assertThrows(
        NullPointerException.class,
        () ->
            new Superstructure(
                null, harness.intake, harness.hopper, harness.shooter, harness.vision));

    harness.close();
    harness.runCycles(1);

    assertEquals(0, periodicCalls.get());
  }

  @Test
  void shootingStatesUseTheirFinalTaskFourRequirementsAndLeaveIntakeHeld() {
    harness.run(harness.superstructure.setStateCmd(Superstate.DRIVE));
    harness.setAlliance(AllianceStationID.Blue1);
    harness.setPose(new Pose2d(1.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.kZero));

    for (Superstate state : new Superstate[] {Superstate.SHOOT_WITH_AIM, Superstate.SHOOT_NO_AIM}) {
      Command command = harness.superstructure.setStateCmd(state);
      assertEquals(
          Set.of(harness.superstructure, harness.hopper, harness.shooter),
          command.getRequirements());

      harness.run(command);

      assertEquals(state, harness.superstructure.getCurrentState());
      assertEquals(IntakeState.DEPLOYED, harness.intake.getDesiredState());
      assertEquals(HopperState.STOP, harness.hopper.getDesiredState());
      assertEquals(ShooterState.SHOOT, harness.shooter.getDesiredState());
    }

    Command manualCommand = harness.superstructure.setStateCmd(Superstate.MANUAL_SHOOT);
    assertEquals(
        Set.of(harness.superstructure, harness.hopper, harness.shooter, harness.drive),
        manualCommand.getRequirements());

    harness.run(manualCommand);

    assertEquals(Superstate.MANUAL_SHOOT, harness.superstructure.getCurrentState());
    assertEquals(IntakeState.DEPLOYED, harness.intake.getDesiredState());
    assertEquals(HopperState.STOP, harness.hopper.getDesiredState());
    assertEquals(ShooterState.SHOOT, harness.shooter.getDesiredState());
  }

  @Test
  void purgeRequestsExactFinalStatesAndRequirementsInPurgeZone() {
    harness.setAlliance(AllianceStationID.Blue1);
    Pose2d unrotated = harness.poseInside(FieldZones.NEUTRAL_LEFT_PURGE);
    Translation2d target = FieldConstants.AimPoints.BLUE_LEFT_PURGE_POINT;
    harness.setPose(
        new Pose2d(
            unrotated.getTranslation(), target.minus(unrotated.getTranslation()).getAngle()));
    Command command = harness.superstructure.setStateCmd(Superstate.PURGE);
    assertEquals(
        Set.of(harness.superstructure, harness.intake, harness.hopper, harness.shooter),
        command.getRequirements());

    harness.run(command);

    assertEquals(Superstate.PURGE, harness.superstructure.getCurrentState());
    assertEquals(IntakeState.OUTTAKE, harness.intake.getDesiredState());
    assertEquals(HopperState.STOP, harness.hopper.getDesiredState());
    assertEquals(ShooterState.SHOOT, harness.shooter.getDesiredState());
  }

  @Test
  void mechanismOverridesRemainAuthoritativeWithOnlyTheirOwnRequirement() {
    Command intakeOverride = harness.superstructure.intakeOverrideCmd(IntakeState.INTAKE);
    Command hopperOverride = harness.superstructure.hopperOverrideCmd(HopperState.INDEX_TO_SHOOTER);
    Command shooterOverride = harness.superstructure.shooterOverrideCmd(ShooterState.STOP);
    assertEquals(Set.of(harness.intake), intakeOverride.getRequirements());
    assertEquals(Set.of(harness.hopper), hopperOverride.getRequirements());
    assertEquals(Set.of(harness.shooter), shooterOverride.getRequirements());

    harness.run(intakeOverride);
    harness.run(hopperOverride);
    harness.run(shooterOverride);
    harness.runCycles(2);

    assertEquals(IntakeState.INTAKE, harness.intake.getDesiredState());
    assertEquals(HopperState.INDEX_TO_SHOOTER, harness.hopper.getDesiredState());
    assertEquals(ShooterState.STOP, harness.shooter.getDesiredState());
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
