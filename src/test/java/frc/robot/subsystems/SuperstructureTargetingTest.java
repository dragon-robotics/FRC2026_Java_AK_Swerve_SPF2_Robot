package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;
import java.util.EnumSet;
import org.junit.jupiter.api.Test;

class SuperstructureTargetingTest {
  private static final EnumSet<FieldZones> ALLOWED =
      EnumSet.of(
          FieldZones.ALLIANCE_LEFT,
          FieldZones.ALLIANCE_RIGHT,
          FieldZones.ALLIANCE_LEFT_TRENCH,
          FieldZones.ALLIANCE_RIGHT_TRENCH,
          FieldZones.NEUTRAL_LEFT_SHOOT,
          FieldZones.NEUTRAL_RIGHT_SHOOT,
          FieldZones.NEUTRAL_LEFT_PURGE,
          FieldZones.NEUTRAL_RIGHT_PURGE);

  private static final EnumSet<FieldZones> PURGE =
      EnumSet.of(FieldZones.NEUTRAL_LEFT_PURGE, FieldZones.NEUTRAL_RIGHT_PURGE);

  private static final EnumSet<FieldZones> NEUTRAL_SHOOT_OR_PURGE =
      EnumSet.of(
          FieldZones.NEUTRAL_LEFT_SHOOT,
          FieldZones.NEUTRAL_RIGHT_SHOOT,
          FieldZones.NEUTRAL_LEFT_PURGE,
          FieldZones.NEUTRAL_RIGHT_PURGE);

  @Test
  void shootAllowedSetIsExactAndRequiresConfirmedAlliance() {
    for (FieldZones zone : FieldZones.values()) {
      assertEquals(
          ALLOWED.contains(zone), SuperstructureTargeting.isShootAllowed(true, zone), zone.name());
      assertFalse(SuperstructureTargeting.isShootAllowed(false, zone), zone.name());
    }
    assertFalse(SuperstructureTargeting.isShootAllowed(true, null));
  }

  @Test
  void purgeAndNeutralSetsAreExactAndRequireConfirmedAlliance() {
    for (FieldZones zone : FieldZones.values()) {
      assertEquals(
          PURGE.contains(zone), SuperstructureTargeting.isPurgeZone(true, zone), zone.name());
      assertEquals(
          NEUTRAL_SHOOT_OR_PURGE.contains(zone),
          SuperstructureTargeting.isNeutralShootOrPurgeZone(true, zone),
          zone.name());
      assertFalse(SuperstructureTargeting.isPurgeZone(false, zone), zone.name());
      assertFalse(SuperstructureTargeting.isNeutralShootOrPurgeZone(false, zone), zone.name());
    }
    assertFalse(SuperstructureTargeting.isPurgeZone(true, null));
    assertFalse(SuperstructureTargeting.isNeutralShootOrPurgeZone(true, null));
  }

  @Test
  void neutralAimPointsKeepAllianceRelativeLeftAndRight() {
    assertEquals(
        FieldConstants.AimPoints.BLUE_LEFT_SHOOT_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_LEFT_SHOOT, Alliance.Blue));
    assertEquals(
        FieldConstants.AimPoints.BLUE_RIGHT_PURGE_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_RIGHT_PURGE, Alliance.Blue));
    assertEquals(
        FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_LEFT_SHOOT, Alliance.Red));
    assertEquals(
        FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_RIGHT_PURGE, Alliance.Red));
  }

  @Test
  void missingAllianceConfirmationAlwaysFallsBackToBlueHub() {
    assertEquals(
        FieldConstants.Hub.BLUE_CENTER_POSE,
        SuperstructureTargeting.resolveAimTarget(false, null, Alliance.Blue));
    assertEquals(
        FieldConstants.Hub.BLUE_CENTER_POSE,
        SuperstructureTargeting.resolveAimTarget(false, null, Alliance.Red));
  }

  @Test
  void confirmedAllianceWithUnknownZoneFallsBackToItsHub() {
    assertEquals(
        FieldConstants.Hub.BLUE_CENTER_POSE,
        SuperstructureTargeting.resolveAimTarget(true, null, Alliance.Blue));
    assertEquals(
        FieldConstants.Hub.RED_CENTER_POSE,
        SuperstructureTargeting.resolveAimTarget(true, null, Alliance.Red));
  }

  @Test
  void geometricHeadingFacesTheTarget() {
    assertEquals(
        Rotation2d.fromDegrees(90.0),
        SuperstructureTargeting.geometricTargetHeading(
            new Pose2d(0.0, 0.0, new Rotation2d()), new Translation2d(0.0, 1.0)));
  }

  @Test
  void alignmentUsesStrictFiveDegreeBoundaryAndWraps() {
    Translation2d target = new Translation2d(1.0, 0.0);
    assertTrue(
        SuperstructureTargeting.isAligned(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(4.999)), target, 5.0));
    assertFalse(
        SuperstructureTargeting.isAligned(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(5.0)), target, 5.0));
    assertTrue(
        SuperstructureTargeting.isAligned(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(359.0)), target, 5.0));
  }
}
