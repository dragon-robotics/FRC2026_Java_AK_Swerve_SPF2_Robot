package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure.Superstate;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SuperstructureCommandRequirementsTest {
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
  void noAimDoesNotRequireDriveButAimedAndManualShotsDo() {
    harness.setAlliance(AllianceStationID.Blue1);
    harness.setPose(new Pose2d(1.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.kZero));

    assertFalse(
        harness
            .superstructure
            .setStateCmd(Superstate.SHOOT_NO_AIM)
            .getRequirements()
            .contains(harness.drive));
    assertFalse(
        harness
            .superstructure
            .setStateCmd(Superstate.SHOOT_NO_AIM)
            .getRequirements()
            .contains(harness.intake));
    assertTrue(
        harness
            .superstructure
            .setStateCmd(Superstate.SHOOT_WITH_AIM)
            .getRequirements()
            .contains(harness.drive));
    assertTrue(
        harness
            .superstructure
            .setStateCmd(Superstate.MANUAL_SHOOT)
            .getRequirements()
            .contains(harness.drive));
    assertFalse(
        harness
            .superstructure
            .setStateCmd(Superstate.MANUAL_SHOOT)
            .getRequirements()
            .contains(harness.intake));
    assertTrue(
        harness.superstructure.shootWithJuicerDelayCmd().getRequirements().contains(harness.drive));
    assertFalse(
        harness
            .superstructure
            .shootNoAimWithJuicerDelayCmd()
            .getRequirements()
            .contains(harness.drive));
  }

  @Test
  void disallowedDefaultShotHasNoRequirementsAndChangesNoMechanisms() {
    harness.setAlliance(AllianceStationID.Blue1);
    harness.setPose(harness.poseInside(FieldZones.ALLIANCE_LEFT_BUMP));
    IntakeState intakeBefore = harness.intake.getDesiredState();
    HopperState hopperBefore = harness.hopper.getDesiredState();
    ShooterState shooterBefore = harness.shooter.getDesiredState();

    Command disallowed = harness.superstructure.selectedShootModeCmd();
    assertTrue(disallowed.getRequirements().isEmpty());
    harness.run(disallowed);

    assertEquals(intakeBefore, harness.intake.getDesiredState());
    assertEquals(hopperBefore, harness.hopper.getDesiredState());
    assertEquals(shooterBefore, harness.shooter.getDesiredState());
  }

  @Test
  void regularAimedAndPurgeShotsOwnDriveAndRequestZeroTranslation() {
    harness.setAlliance(AllianceStationID.Blue1);
    harness.setPose(
        new Pose2d(1.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.fromDegrees(20.0)));
    Command aimed = harness.superstructure.setStateCmd(Superstate.SHOOT_WITH_AIM);
    assertTrue(aimed.getRequirements().contains(harness.drive));
    assertFalse(aimed.getRequirements().contains(harness.intake));
    harness.run(harness.superstructure.setStateCmd(Superstate.DRIVE));
    assertEquals(IntakeState.DEPLOYED, harness.intake.getDesiredState());
    harness.run(aimed);
    assertEquals(
        IntakeState.DEPLOYED,
        harness.intake.getDesiredState(),
        "regular shooting must preserve the prior Intake request");
    assertEquals(0.0, harness.drive.lastRequestedSpeeds().vxMetersPerSecond, 1e-9);
    assertEquals(0.0, harness.drive.lastRequestedSpeeds().vyMetersPerSecond, 1e-9);

    aimed.cancel();
    harness.setPose(harness.poseInside(FieldZones.NEUTRAL_LEFT_PURGE));
    Command purge = harness.superstructure.purgeShootCmd();
    assertTrue(purge.getRequirements().contains(harness.drive));
    assertTrue(purge.getRequirements().contains(harness.intake));
    assertTrue(
        harness
            .superstructure
            .setStateCmd(Superstate.PURGE)
            .getRequirements()
            .contains(harness.drive));
    harness.run(purge);
    assertEquals(0.0, harness.drive.lastRequestedSpeeds().vxMetersPerSecond, 1e-9);
    assertEquals(0.0, harness.drive.lastRequestedSpeeds().vyMetersPerSecond, 1e-9);
  }
}
