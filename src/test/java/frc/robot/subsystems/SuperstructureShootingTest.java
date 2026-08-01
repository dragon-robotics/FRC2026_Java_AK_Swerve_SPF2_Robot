package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure.ShootMode;
import frc.robot.subsystems.Superstructure.Superstate;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SuperstructureShootingTest {
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
  void automaticFeedRequiresShooterShootStateAndAlignment() {
    harness.setAlliance(AllianceStationID.Blue1);
    harness.setPose(new Pose2d(1.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.kZero));
    Command command = harness.superstructure.setStateCmd(Superstate.SHOOT_NO_AIM);
    harness.run(command);
    assertEquals(HopperState.STOP, harness.hopper.getDesiredState());

    harness.makeShooterReady();
    CommandScheduler.getInstance().run();
    assertEquals(HopperState.INDEX_TO_SHOOTER, harness.hopper.getDesiredState());

    harness.setPose(new Pose2d(1.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.fromDegrees(5.0)));
    harness.superstructure.periodic();
    CommandScheduler.getInstance().run();
    assertEquals(HopperState.STOP, harness.hopper.getDesiredState());
  }

  @Test
  void aimedFeedAlsoRequiresAlignmentAndReportsVisionAiming() {
    harness.setAlliance(AllianceStationID.Blue1);
    harness.setPose(new Pose2d(1.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.fromDegrees(6.0)));

    harness.run(harness.superstructure.setStateCmd(Superstate.SHOOT_WITH_AIM));
    harness.makeShooterReady();
    harness.runCycles(1);
    harness.superstructure.periodic();

    assertEquals(HopperState.STOP, harness.hopper.getDesiredState());
    assertTrue(harness.vision.aiming);
  }

  @Test
  void manualModesUseApprovedPresetsAndIgnoreAlignment() {
    harness.setPose(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(90.0)));
    harness.run(harness.superstructure.setShootModeCmd(ShootMode.MANUAL_BUMPER_UP));
    Command bumperCommand = harness.superstructure.selectedShootModeCmd();
    assertEquals(
        Set.of(harness.superstructure, harness.shooter, harness.hopper, harness.drive),
        bumperCommand.getRequirements());
    harness.drive.runVelocity(new ChassisSpeeds(1.0, -1.0, 0.5));
    harness.run(bumperCommand);
    harness.superstructure.periodic();
    assertEquals(2500.0, harness.shooter.getTargetRpm());
    assertEquals(0.0, harness.shooter.getTargetHoodRotations());
    assertEquals(0.0, harness.drive.lastRequestedSpeeds().vxMetersPerSecond);
    harness.makeShooterReady();
    harness.runCycles(1);
    assertEquals(HopperState.INDEX_TO_SHOOTER, harness.hopper.getDesiredState());

    harness.run(harness.superstructure.setShootModeCmd(ShootMode.MANUAL_TRENCH));
    harness.run(harness.superstructure.selectedShootModeCmd());
    harness.superstructure.periodic();
    assertEquals(2900.0, harness.shooter.getTargetRpm());
    assertEquals(0.75, harness.shooter.getTargetHoodRotations());
  }

  @Test
  void readyShooterFeedsAcrossNeutralZoneTargetRefresh() {
    harness.setAlliance(AllianceStationID.Blue1);
    Pose2d unrotated = harness.poseInside(FieldZones.NEUTRAL_LEFT_SHOOT);
    Translation2d target = FieldConstants.AimPoints.BLUE_LEFT_SHOOT_POINT;
    Pose2d aligned =
        new Pose2d(unrotated.getTranslation(), target.minus(unrotated.getTranslation()).getAngle());
    harness.setPose(aligned);
    assertTrue(harness.superstructure.isAlignedToTarget());

    Command command = harness.superstructure.setStateCmd(Superstate.SHOOT_NO_AIM);
    harness.run(command);
    harness.makeShooterReady();
    assertEquals(ShooterState.SHOOT, harness.shooter.getCurrentState());

    CommandScheduler.getInstance().run();

    assertEquals(ShooterState.SHOOT, harness.shooter.getCurrentState());
    assertEquals(HopperState.INDEX_TO_SHOOTER, harness.hopper.getDesiredState());
  }

  @Test
  void everyNeutralShootAndPurgeZoneCapsHoodWithoutChangingInterpolatedRpm() {
    harness.setAlliance(AllianceStationID.Blue1);
    for (FieldZones zone :
        List.of(
            FieldZones.NEUTRAL_LEFT_SHOOT,
            FieldZones.NEUTRAL_RIGHT_SHOOT,
            FieldZones.NEUTRAL_LEFT_PURGE,
            FieldZones.NEUTRAL_RIGHT_PURGE)) {
      Pose2d pose = harness.poseInside(zone);
      harness.setPose(pose);
      Shooter interpolationReference = new Shooter(new ShooterIO.NoOp());
      Translation2d aimTarget = SuperstructureTargeting.resolveAimTarget(true, zone, Alliance.Blue);
      interpolationReference.setSetpointForDistance(pose.getTranslation().getDistance(aimTarget));
      double expectedRpm = interpolationReference.getTargetRpm();
      harness.superstructure.periodic();
      assertEquals(expectedRpm, harness.shooter.getTargetRpm(), 1e-9, zone.name());
      assertEquals(1.25, harness.shooter.getTargetHoodRotations(), zone.name());
    }
  }

  @Test
  void periodicCachesAimTargetAndUsesItsDistanceForShooterSetpoint() {
    harness.setAlliance(AllianceStationID.Blue1);
    Translation2d target = FieldConstants.Hub.BLUE_CENTER_POSE;
    Translation2d robotTranslation = target.minus(new Translation2d(0.9144, 1.2192));
    harness.setPose(new Pose2d(robotTranslation, target.minus(robotTranslation).getAngle()));

    assertEquals(target, harness.superstructure.getCurrentAimTarget());
    assertEquals(1.524, harness.drive.getPose().getTranslation().getDistance(target), 1e-9);
    assertEquals(2400.0, harness.shooter.getTargetRpm());
    assertEquals(0.0, harness.shooter.getTargetHoodRotations());
    assertTrue(harness.superstructure.isAlignedToTarget());
  }

  @Test
  void modeFactoriesToggleAtInvocationAndRejectNullImmediately() {
    assertEquals(ShootMode.DEFAULT_SHOOT_WITH_AIM, harness.superstructure.getShootMode());
    harness.run(harness.superstructure.toggleShootModeCmd(ShootMode.MANUAL_BUMPER_UP));
    assertEquals(ShootMode.MANUAL_BUMPER_UP, harness.superstructure.getShootMode());
    harness.run(harness.superstructure.toggleShootModeCmd(ShootMode.MANUAL_BUMPER_UP));
    assertEquals(ShootMode.DEFAULT_SHOOT_WITH_AIM, harness.superstructure.getShootMode());
    harness.run(harness.superstructure.setShootModeCmd(ShootMode.MANUAL_TRENCH));
    assertEquals(ShootMode.MANUAL_TRENCH, harness.superstructure.getShootMode());

    assertThrows(NullPointerException.class, () -> harness.superstructure.setShootModeCmd(null));
    assertThrows(NullPointerException.class, () -> harness.superstructure.toggleShootModeCmd(null));
  }

  @Test
  void disallowedAutomaticCommandsOwnNothingAndChangeNothing() {
    harness.setAlliance(AllianceStationID.Blue1);
    harness.setPose(harness.poseInside(FieldZones.OPPONENT_LEFT));

    for (Superstate state :
        List.of(Superstate.SHOOT_WITH_AIM, Superstate.SHOOT_NO_AIM, Superstate.PURGE)) {
      Command command = harness.superstructure.setStateCmd(state);
      assertTrue(command.getRequirements().isEmpty(), state.name());
      harness.run(command);
      assertEquals(Superstate.DRIVE_STARTING_CONFIG, harness.superstructure.getCurrentState());
      assertEquals(IntakeState.HOME, harness.intake.getDesiredState());
      assertEquals(HopperState.STOP, harness.hopper.getDesiredState());
      assertEquals(ShooterState.PREPFUEL, harness.shooter.getDesiredState());
    }
  }

  @Test
  void unconfirmedAllianceDefaultsBlueTargetButDisallowsAutomaticShooting() {
    DriverStationSim.resetData();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    harness.setPose(new Pose2d(1.0, 1.0, Rotation2d.kZero));

    assertEquals(FieldConstants.Hub.BLUE_CENTER_POSE, harness.superstructure.getCurrentAimTarget());
    assertFalse(harness.superstructure.isShootAllowed());
    assertFalse(harness.superstructure.isSelectedShootAllowed());
    assertTrue(
        harness.superstructure.setStateCmd(Superstate.SHOOT_WITH_AIM).getRequirements().isEmpty());

    harness.run(harness.superstructure.setShootModeCmd(ShootMode.MANUAL_TRENCH));
    assertTrue(harness.superstructure.isSelectedShootAllowed());
  }

  @Test
  void purgeZoneRoutesDefaultShootAndExplicitPurgeThroughAlignedPurgeMechanism() {
    harness.setAlliance(AllianceStationID.Blue1);
    Pose2d unrotated = harness.poseInside(FieldZones.NEUTRAL_LEFT_PURGE);
    Translation2d target = FieldConstants.AimPoints.BLUE_LEFT_PURGE_POINT;
    Pose2d aligned =
        new Pose2d(unrotated.getTranslation(), target.minus(unrotated.getTranslation()).getAngle());
    harness.setPose(aligned);

    assertTrue(harness.superstructure.shouldUsePurgeDuringShoot());
    Command selected = harness.superstructure.selectedShootModeCmd();
    assertEquals(
        Set.of(
            harness.superstructure, harness.intake, harness.shooter, harness.hopper, harness.drive),
        selected.getRequirements());
    harness.run(selected);
    assertEquals(Superstate.PURGE, harness.superstructure.getCurrentState());
    assertEquals(IntakeState.OUTTAKE, harness.intake.getDesiredState());
    harness.makeShooterReady();
    selected.execute();
    assertEquals(HopperState.INDEX_TO_SHOOTER, harness.hopper.getDesiredState());

    CommandScheduler.getInstance().cancelAll();
    Command explicitPurge = harness.superstructure.purgeShootCmd();
    assertEquals(
        Set.of(
            harness.superstructure, harness.intake, harness.shooter, harness.hopper, harness.drive),
        explicitPurge.getRequirements());
  }

  @Test
  void visionAimingIsTrueOnlyForAutomaticShootStates() {
    harness.setAlliance(AllianceStationID.Blue1);
    harness.setPose(new Pose2d(1.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.kZero));

    for (Superstate state :
        List.of(
            Superstate.SHOOT_WITH_AIM,
            Superstate.SHOOT_NO_AIM,
            Superstate.DRIVE,
            Superstate.MANUAL_SHOOT)) {
      harness.run(harness.superstructure.setStateCmd(state));
      harness.superstructure.periodic();
      assertEquals(
          state == Superstate.SHOOT_WITH_AIM || state == Superstate.SHOOT_NO_AIM,
          harness.vision.aiming,
          state.name());
    }

    harness.setPose(harness.poseInside(FieldZones.NEUTRAL_LEFT_PURGE));
    harness.run(harness.superstructure.setStateCmd(Superstate.PURGE));
    harness.superstructure.periodic();
    assertFalse(harness.vision.aiming);
  }
}
