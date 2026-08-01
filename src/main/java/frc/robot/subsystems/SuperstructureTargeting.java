package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;

/** Pure field-targeting rules used by the superstructure. */
final class SuperstructureTargeting {
  private SuperstructureTargeting() {}

  static Translation2d resolveAimTarget(
      boolean allianceConfirmed, FieldZones zone, Alliance alliance) {
    if (!allianceConfirmed) {
      return FieldConstants.Hub.BLUE_CENTER_POSE;
    }
    if (zone == null) {
      return alliance == Alliance.Red
          ? FieldConstants.Hub.RED_CENTER_POSE
          : FieldConstants.Hub.BLUE_CENTER_POSE;
    }
    boolean red = alliance == Alliance.Red;
    return switch (zone) {
      case NEUTRAL_LEFT_SHOOT -> red
          ? FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT
          : FieldConstants.AimPoints.BLUE_LEFT_SHOOT_POINT;
      case NEUTRAL_RIGHT_SHOOT -> red
          ? FieldConstants.AimPoints.RED_RIGHT_SHOOT_POINT
          : FieldConstants.AimPoints.BLUE_RIGHT_SHOOT_POINT;
      case NEUTRAL_LEFT_PURGE -> red
          ? FieldConstants.AimPoints.RED_LEFT_PURGE_POINT
          : FieldConstants.AimPoints.BLUE_LEFT_PURGE_POINT;
      case NEUTRAL_RIGHT_PURGE -> red
          ? FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT
          : FieldConstants.AimPoints.BLUE_RIGHT_PURGE_POINT;
      default -> red ? FieldConstants.Hub.RED_CENTER_POSE : FieldConstants.Hub.BLUE_CENTER_POSE;
    };
  }

  static boolean isShootAllowed(boolean allianceConfirmed, FieldZones zone) {
    if (!allianceConfirmed || zone == null) return false;
    return switch (zone) {
      case ALLIANCE_LEFT,
          ALLIANCE_RIGHT,
          ALLIANCE_LEFT_TRENCH,
          ALLIANCE_RIGHT_TRENCH,
          NEUTRAL_LEFT_SHOOT,
          NEUTRAL_RIGHT_SHOOT,
          NEUTRAL_LEFT_PURGE,
          NEUTRAL_RIGHT_PURGE -> true;
      default -> false;
    };
  }

  static boolean isPurgeZone(boolean allianceConfirmed, FieldZones zone) {
    return allianceConfirmed
        && (zone == FieldZones.NEUTRAL_LEFT_PURGE || zone == FieldZones.NEUTRAL_RIGHT_PURGE);
  }

  static boolean isNeutralShootOrPurgeZone(boolean allianceConfirmed, FieldZones zone) {
    return allianceConfirmed
        && (zone == FieldZones.NEUTRAL_LEFT_SHOOT
            || zone == FieldZones.NEUTRAL_RIGHT_SHOOT
            || zone == FieldZones.NEUTRAL_LEFT_PURGE
            || zone == FieldZones.NEUTRAL_RIGHT_PURGE);
  }

  static Rotation2d geometricTargetHeading(Pose2d pose, Translation2d target) {
    return new Rotation2d(Math.atan2(target.getY() - pose.getY(), target.getX() - pose.getX()));
  }

  static boolean isAligned(Pose2d pose, Translation2d target, double toleranceDegrees) {
    double error =
        Math.IEEEremainder(
            pose.getRotation().getRadians() - geometricTargetHeading(pose, target).getRadians(),
            2.0 * Math.PI);
    return Math.abs(Math.toDegrees(error)) < toleranceDegrees;
  }
}
