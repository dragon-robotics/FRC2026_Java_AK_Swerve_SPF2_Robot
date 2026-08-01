package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.util.constants.FieldConstants.FieldZones;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SuperstructureAutonomousTest {
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
  void autonomousShootChangesToJuicerAtInclusiveOnePointFiveSeconds() {
    Command command = harness.superstructure.shootNoAimWithJuicerDelayCmd();
    harness.run(command);
    assertEquals(IntakeState.DEPLOYED, harness.intake.getDesiredState());

    harness.fixedTimer.setTime(1.499);
    CommandScheduler.getInstance().run();
    assertEquals(IntakeState.DEPLOYED, harness.intake.getDesiredState());

    harness.fixedTimer.setTime(1.500);
    CommandScheduler.getInstance().run();
    assertEquals(IntakeState.JUICER, harness.intake.getDesiredState());
  }

  @Test
  void aimedAutonomousPurgeUsesOuttakeForTheEntireRun() {
    harness.setAlliance(AllianceStationID.Blue1);
    harness.setPose(harness.poseInside(FieldZones.NEUTRAL_LEFT_PURGE));

    Command command = harness.superstructure.shootWithJuicerDelayCmd();
    harness.run(command);
    assertEquals(IntakeState.OUTTAKE, harness.intake.getDesiredState());

    harness.fixedTimer.setTime(1.500);
    CommandScheduler.getInstance().run();
    assertEquals(IntakeState.OUTTAKE, harness.intake.getDesiredState());

    harness.fixedTimer.setTime(5.0);
    CommandScheduler.getInstance().run();
    assertEquals(IntakeState.OUTTAKE, harness.intake.getDesiredState());
  }

  @Test
  void headingStateZoneLockAndVisionReseedExposeTheRequiredContainerSeams() {
    harness.setAlliance(AllianceStationID.Blue1);
    harness.setPose(harness.poseInside(FieldZones.ALLIANCE_LEFT));
    Optional<Rotation2d> heading = Optional.of(Rotation2d.fromDegrees(17.0));
    harness.superstructure.setCurrentHeading(heading);
    harness.superstructure.setRotationLastTriggered(12.5);

    assertEquals(heading, harness.superstructure.getCurrentHeading());
    assertEquals(12.5, harness.superstructure.getRotationLastTriggered(), 1e-9);
    assertEquals(
        DriveCommands.getZoneLockedHeading(FieldZones.ALLIANCE_LEFT, Alliance.Blue),
        harness.superstructure.getZoneLockedHeading());

    Command reseed = harness.superstructure.forceReseedFromVisionCmd();
    assertTrue(reseed.getRequirements().contains(harness.vision));
    assertFalse(reseed.getRequirements().contains(harness.drive));
    harness.run(reseed);
    assertEquals(1, harness.vision.reseedCalls);
  }
}
