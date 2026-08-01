package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;
import org.junit.jupiter.api.Test;

class SuperstructureAimTargetTest {
  @Test
  void redAllianceFacingGeometricTargetIsAligned() {
    Translation2d target = new Translation2d(1.0, 0.0);
    Pose2d poseFacingGeometricTarget = new Pose2d(0.0, 0.0, Rotation2d.kZero);

    assertTrue(SuperstructureTargeting.isAligned(poseFacingGeometricTarget, target, 5.0));
  }

  @Test
  void geometricAlignmentIsSameForBlueAndRed() {
    Translation2d target = new Translation2d(1.0, 0.0);
    Pose2d poseFacingTarget = new Pose2d(0.0, 0.0, Rotation2d.kZero);
    Pose2d poseFacingAway = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0));

    for (Alliance alliance : Alliance.values()) {
      assertTrue(SuperstructureTargeting.isAligned(poseFacingTarget, target, 5.0), alliance.name());
      assertFalse(SuperstructureTargeting.isAligned(poseFacingAway, target, 5.0), alliance.name());
    }
  }

  @Test
  void rawGeometricHeadingDoesNotAddRedOperatorPerspectivePi() {
    Pose2d pose = new Pose2d(2.0, 3.0, Rotation2d.kZero);
    Translation2d target = new Translation2d(6.0, 7.0);
    Rotation2d directAtan2 = new Rotation2d(Math.atan2(4.0, 4.0));

    for (Alliance alliance : Alliance.values()) {
      assertEquals(
          directAtan2,
          SuperstructureTargeting.geometricTargetHeading(pose, target),
          alliance.name());
    }
  }

  @Test
  void redRightAimPointsMirrorRedLeftAcrossHorizontalCenterline() {
    assertEquals(
        FieldConstants.FIELD_WIDTH - FieldConstants.AimPoints.RED_LEFT_PURGE_POINT.getY(),
        FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT.getY());
    assertEquals(
        FieldConstants.FIELD_WIDTH - FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT.getY(),
        FieldConstants.AimPoints.RED_RIGHT_SHOOT_POINT.getY());
    assertNotEquals(
        FieldConstants.AimPoints.RED_LEFT_PURGE_POINT,
        FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT);
    assertNotEquals(
        FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT,
        FieldConstants.AimPoints.RED_RIGHT_SHOOT_POINT);
  }

  @Test
  void redAllianceNeutralZonesUseDirectLeftRightAimPoints() {
    assertEquals(
        FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_LEFT_SHOOT, Alliance.Red));
    assertEquals(
        FieldConstants.AimPoints.RED_RIGHT_SHOOT_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_RIGHT_SHOOT, Alliance.Red));
    assertEquals(
        FieldConstants.AimPoints.RED_LEFT_PURGE_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_LEFT_PURGE, Alliance.Red));
    assertEquals(
        FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_RIGHT_PURGE, Alliance.Red));
  }

  @Test
  void blueAllianceNeutralZonesKeepStandardLeftRightAimPoints() {
    assertEquals(
        FieldConstants.AimPoints.BLUE_LEFT_SHOOT_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_LEFT_SHOOT, Alliance.Blue));
    assertEquals(
        FieldConstants.AimPoints.BLUE_RIGHT_SHOOT_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_RIGHT_SHOOT, Alliance.Blue));
    assertEquals(
        FieldConstants.AimPoints.BLUE_LEFT_PURGE_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_LEFT_PURGE, Alliance.Blue));
    assertEquals(
        FieldConstants.AimPoints.BLUE_RIGHT_PURGE_POINT,
        SuperstructureTargeting.resolveAimTarget(
            true, FieldZones.NEUTRAL_RIGHT_PURGE, Alliance.Blue));
  }

  @Test
  void defaultsToAllianceHubWhenZoneUnknownAndBlueWhenUnconfirmed() {
    assertEquals(
        FieldConstants.Hub.RED_CENTER_POSE,
        SuperstructureTargeting.resolveAimTarget(true, null, Alliance.Red));
    assertEquals(
        FieldConstants.Hub.BLUE_CENTER_POSE,
        SuperstructureTargeting.resolveAimTarget(true, null, Alliance.Blue));
    assertEquals(
        FieldConstants.Hub.BLUE_CENTER_POSE,
        SuperstructureTargeting.resolveAimTarget(
            false, FieldZones.NEUTRAL_LEFT_SHOOT, Alliance.Red));
    assertEquals(
        FieldConstants.Hub.BLUE_CENTER_POSE,
        SuperstructureTargeting.resolveAimTarget(
            false, FieldZones.NEUTRAL_RIGHT_PURGE, Alliance.Blue));
  }
}
