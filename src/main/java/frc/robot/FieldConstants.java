package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.drive.DriveConstants;
import java.io.IOException;
import java.nio.file.Path;

/**
 * Field geometry, AprilTag layout, and derived field-zone helpers.
 *
 * <p>All field distances use meters in the WPILib field coordinate frame. Zone classification
 * normalizes poses to the blue-alliance perspective before applying boundaries.
 */
public final class FieldConstants {

  /** AprilTag field layout loaded from deploy, with WPILib default field fallback. */
  public static final AprilTagFieldLayout APTAG_FIELD_LAYOUT = loadDefaultAprilTagLayout();

  private static AprilTagFieldLayout loadDefaultAprilTagLayout() {
    Path defaultPath =
        Path.of(
            Filesystem.getDeployDirectory().getPath(),
            "apriltags",
            "welded",
            "2026-rebuilt-welded-no-single.json");
    AprilTagFieldLayout defaultLayout = null;
    try {
      defaultLayout = new AprilTagFieldLayout(defaultPath);
    } catch (IOException e) {
      System.err.println("!!! CRITICAL: Failed to load default AprilTag field resource!");
      DriverStation.reportError(
          "CRITICAL: Failed to load default AprilTag field resource: " + e.getMessage(), true);
      defaultLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
    return defaultLayout;
  }

  /** Field length in meters from the active AprilTag layout. */
  public static final double FIELD_LENGTH = APTAG_FIELD_LAYOUT.getFieldLength();

  /** Field width in meters from the active AprilTag layout. */
  public static final double FIELD_WIDTH = APTAG_FIELD_LAYOUT.getFieldWidth();

  /** Approximate playable field height in meters. */
  public static final double FIELD_HEIGHT = Units.inchesToMeters(72);

  /** Named field zones used by autonomous pathing, aiming, and purge decisions. */
  public static enum FieldZones {
    ALLIANCE_LEFT,
    ALLIANCE_LEFT_TRENCH,
    ALLIANCE_LEFT_BUMP,
    ALLIANCE_RIGHT,
    ALLIANCE_RIGHT_TRENCH,
    ALLIANCE_RIGHT_BUMP,
    NEUTRAL_LEFT,
    NEUTRAL_LEFT_SHOOT,
    NEUTRAL_LEFT_PURGE,
    NEUTRAL_RIGHT,
    NEUTRAL_RIGHT_SHOOT,
    NEUTRAL_RIGHT_PURGE,
    OPPONENT_LEFT,
    OPPONENT_LEFT_TRENCH,
    OPPONENT_LEFT_BUMP,
    OPPONENT_RIGHT,
    OPPONENT_RIGHT_TRENCH,
    OPPONENT_RIGHT_BUMP;

    /**
     * Classifies a robot pose into a named field zone.
     *
     * <p>Red-alliance poses are mirrored into the blue-alliance frame before classification, so
     * left/right labels are relative to the active alliance driver-station perspective.
     */
    public static FieldZones fromPose(Pose2d pose, DriverStation.Alliance alliance) {
      double x = pose.getX();
      double y = pose.getY();

      // Zone boundaries are authored from the blue-alliance perspective.
      double normalizedX = alliance == DriverStation.Alliance.Red ? FIELD_LENGTH - x : x;
      double normalizedY = alliance == DriverStation.Alliance.Red ? FIELD_WIDTH - y : y;

      boolean isLeft = normalizedY > LinesHorizontal.CENTER;

      if (normalizedX < allianceZoneBoundaryX()) {
        return isLeft ? ALLIANCE_LEFT : ALLIANCE_RIGHT;
      }

      if (normalizedX < nearNeutralZoneBoundaryX()) {
        return classifyNearNeutralZone(isLeft, normalizedY);
      }

      FieldZones neutralSpecialZone = classifyNeutralSpecialZone(normalizedX, normalizedY);
      if (neutralSpecialZone != null) {
        return neutralSpecialZone;
      }

      if (normalizedX < farNeutralZoneBoundaryX()) {
        return isLeft ? NEUTRAL_LEFT : NEUTRAL_RIGHT;
      }

      if (normalizedX > opponentAllianceZoneBoundaryX()) {
        return isLeft ? OPPONENT_LEFT : OPPONENT_RIGHT;
      }

      return classifyOpponentZone(isLeft, normalizedY);
    }

    private static FieldZones classifyNeutralSpecialZone(double normalizedX, double normalizedY) {
      if (!isWithinNeutralShootXBand(normalizedX)) {
        return null;
      }

      if (isWithinNeutralPurgeXBand(normalizedX) && isInNeutralLeftPurgeBand(normalizedY)) {
        return NEUTRAL_LEFT_PURGE;
      }
      if (isInNeutralLeftShootBand(normalizedY)) {
        return NEUTRAL_LEFT_SHOOT;
      }
      if (isWithinNeutralPurgeXBand(normalizedX) && isInNeutralRightPurgeBand(normalizedY)) {
        return NEUTRAL_RIGHT_PURGE;
      }
      if (isInNeutralRightShootBand(normalizedY)) {
        return NEUTRAL_RIGHT_SHOOT;
      }

      return null;
    }

    private static boolean isWithinNeutralShootXBand(double normalizedX) {
      return normalizedX >= neutralShootBandMinX() && normalizedX <= neutralShootBandMaxX();
    }

    private static boolean isWithinNeutralPurgeXBand(double normalizedX) {
      return normalizedX >= neutralPurgeBandMinX() && normalizedX <= neutralPurgeBandMaxX();
    }

    private static boolean isInNeutralRightPurgeBand(double normalizedY) {
      return normalizedY >= LinesHorizontal.BLUE_RIGHT_TRENCH_OPEN_END
          && normalizedY
              <= LinesHorizontal.BLUE_RIGHT_TRENCH_OPEN_START
                  - DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS;
    }

    private static boolean isInNeutralRightShootBand(double normalizedY) {
      return normalizedY >= LinesHorizontal.BLUE_RIGHT_TRENCH_OPEN_END
          && normalizedY <= LinesHorizontal.BLUE_RIGHT_BUMP_START;
    }

    private static boolean isInNeutralLeftPurgeBand(double normalizedY) {
      return normalizedY
              >= LinesHorizontal.BLUE_LEFT_TRENCH_OPEN_END
                  + DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS
          && normalizedY <= LinesHorizontal.BLUE_LEFT_TRENCH_OPEN_START;
    }

    private static boolean isInNeutralLeftShootBand(double normalizedY) {
      return normalizedY >= LinesHorizontal.BLUE_LEFT_BUMP_END
          && normalizedY <= LinesHorizontal.BLUE_LEFT_TRENCH_OPEN_START;
    }

    private static double allianceZoneBoundaryX() {
      return LinesVertical.ALLIANCE_ZONE
          - DriveConstants.ROBOT_CENTER_TO_CORNER_WITH_BUMPERS_METERS;
    }

    private static double nearNeutralZoneBoundaryX() {
      return LinesVertical.NEUTRAL_ZONE_NEAR
          + DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS;
    }

    private static double neutralShootBandMinX() {
      return LinesVertical.NEUTRAL_ZONE_NEAR
          + DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS;
    }

    private static double neutralShootBandMaxX() {
      return LinesVertical.CENTER - DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS;
    }

    private static double neutralPurgeBandMinX() {
      return LinesVertical.NEUTRAL_ZONE_NEAR
          + DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS;
    }

    private static double neutralPurgeBandMaxX() {
      return (LinesVertical.NEUTRAL_ZONE_NEAR + LinesVertical.CENTER) / 2.0;
    }

    private static double farNeutralZoneBoundaryX() {
      return LinesVertical.NEUTRAL_ZONE_FAR
          - DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS;
    }

    private static double opponentAllianceZoneBoundaryX() {
      return LinesVertical.OPP_ALLIANCE_ZONE
          + DriveConstants.ROBOT_CENTER_TO_CORNER_WITH_BUMPERS_METERS;
    }

    private static FieldZones classifyOpponentZone(boolean isLeft, double normalizedY) {
      return isLeft
          ? classifyOpponentLeftZone(normalizedY)
          : classifyOpponentRightZone(normalizedY);
    }

    private static FieldZones classifyOpponentLeftZone(double normalizedY) {
      if (normalizedY
              > LinesHorizontal.BLUE_LEFT_TRENCH_OPEN_END
                  + DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS
          && normalizedY <= LinesHorizontal.BLUE_LEFT_TRENCH_OPEN_START) {
        return OPPONENT_LEFT_TRENCH;
      }
      if (normalizedY >= LinesHorizontal.BLUE_LEFT_BUMP_END
          && normalizedY <= LinesHorizontal.BLUE_LEFT_BUMP_START) {
        return OPPONENT_LEFT_BUMP;
      }
      return OPPONENT_LEFT;
    }

    private static FieldZones classifyOpponentRightZone(double normalizedY) {
      if (normalizedY >= LinesHorizontal.BLUE_RIGHT_BUMP_END
          && normalizedY <= LinesHorizontal.BLUE_RIGHT_BUMP_START) {
        return OPPONENT_RIGHT_BUMP;
      }
      if (normalizedY >= LinesHorizontal.BLUE_RIGHT_TRENCH_OPEN_END
          && normalizedY
              <= LinesHorizontal.BLUE_RIGHT_TRENCH_OPEN_START
                  - DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS) {
        return OPPONENT_RIGHT_TRENCH;
      }
      return OPPONENT_RIGHT;
    }

    private static FieldZones classifyNearNeutralZone(boolean isLeft, double normalizedY) {
      return isLeft
          ? classifyLeftNearNeutralZone(normalizedY)
          : classifyRightNearNeutralZone(normalizedY);
    }

    private static FieldZones classifyLeftNearNeutralZone(double normalizedY) {
      if (normalizedY
              > LinesHorizontal.BLUE_LEFT_TRENCH_OPEN_END
                  + DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS
          && normalizedY <= LinesHorizontal.BLUE_LEFT_TRENCH_OPEN_START) {
        return ALLIANCE_LEFT_TRENCH;
      }
      if (normalizedY >= LinesHorizontal.BLUE_LEFT_BUMP_END
          && normalizedY <= LinesHorizontal.BLUE_LEFT_BUMP_START) {
        return ALLIANCE_LEFT_BUMP;
      }
      return ALLIANCE_LEFT;
    }

    private static FieldZones classifyRightNearNeutralZone(double normalizedY) {
      if (normalizedY >= LinesHorizontal.BLUE_RIGHT_BUMP_END
          && normalizedY <= LinesHorizontal.BLUE_RIGHT_BUMP_START) {
        return ALLIANCE_RIGHT_BUMP;
      }
      if (normalizedY >= LinesHorizontal.BLUE_RIGHT_TRENCH_OPEN_END
          && normalizedY
              <= LinesHorizontal.BLUE_RIGHT_TRENCH_OPEN_START
                  - DriveConstants.ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS) {
        return ALLIANCE_RIGHT_TRENCH;
      }
      return ALLIANCE_RIGHT;
    }
  }

  /** Alliance-specific target points for shooting and purging from neutral-field positions. */
  public static class AimPoints {

    public static final Translation2d BLUE_HUB_CENTER = Hub.BLUE_CENTER_POSE;
    public static final Translation2d RED_HUB_CENTER = Hub.RED_CENTER_POSE;

    private static final double BLUE_NEUTRAL_AIM_X = LinesVertical.ALLIANCE_ZONE * 0.90;
    private static final double RED_NEUTRAL_AIM_X = FIELD_LENGTH - BLUE_NEUTRAL_AIM_X;

    /* Blue alliance */
    public static final Translation2d BLUE_LEFT_PURGE_POINT =
        new Translation2d(
            BLUE_NEUTRAL_AIM_X,
            (LinesHorizontal.BLUE_LEFT_TRENCH_OPEN_START
                    + LinesHorizontal.BLUE_LEFT_TRENCH_OPEN_END)
                / 2.0);
    public static final Translation2d BLUE_RIGHT_PURGE_POINT =
        new Translation2d(
            BLUE_NEUTRAL_AIM_X,
            (LinesHorizontal.BLUE_RIGHT_TRENCH_OPEN_START
                    + LinesHorizontal.BLUE_RIGHT_TRENCH_OPEN_END)
                / 2.0);
    public static final Translation2d BLUE_LEFT_SHOOT_POINT =
        new Translation2d(
            BLUE_NEUTRAL_AIM_X,
            (LinesHorizontal.BLUE_LEFT_TRENCH_OPEN_END + LinesHorizontal.CENTER) / 2.0);
    public static final Translation2d BLUE_RIGHT_SHOOT_POINT =
        new Translation2d(
            BLUE_NEUTRAL_AIM_X,
            (LinesHorizontal.BLUE_RIGHT_TRENCH_OPEN_START + LinesHorizontal.CENTER) / 2.0);

    /* Red alliance */
    public static final Translation2d RED_LEFT_PURGE_POINT =
        new Translation2d(
            RED_NEUTRAL_AIM_X,
            FIELD_WIDTH
                - ((LinesHorizontal.RED_LEFT_TRENCH_OPEN_START
                        + LinesHorizontal.RED_LEFT_TRENCH_OPEN_END)
                    / 2.0));
    public static final Translation2d RED_RIGHT_PURGE_POINT =
        new Translation2d(RED_NEUTRAL_AIM_X, FIELD_WIDTH - RED_LEFT_PURGE_POINT.getY());
    public static final Translation2d RED_LEFT_SHOOT_POINT =
        new Translation2d(
            RED_NEUTRAL_AIM_X,
            FIELD_WIDTH
                - ((LinesHorizontal.RED_LEFT_TRENCH_OPEN_END + LinesHorizontal.CENTER) / 2.0));
    public static final Translation2d RED_RIGHT_SHOOT_POINT =
        new Translation2d(RED_NEUTRAL_AIM_X, FIELD_WIDTH - RED_LEFT_SHOOT_POINT.getY());
  }

  /** Relevant field lines defined by X-axis offset in meters. */
  public static class LinesVertical {
    public static final double CENTER = FIELD_LENGTH / 2.0;
    public static final double STARTING = APTAG_FIELD_LAYOUT.getTagPose(26).get().getX();
    public static final double ALLIANCE_ZONE = STARTING;
    public static final double HUB_CENTER =
        APTAG_FIELD_LAYOUT.getTagPose(26).get().getX() + Hub.WIDTH / 2.0;
    public static final double NEUTRAL_ZONE_NEAR = CENTER - Units.inchesToMeters(120);
    public static final double NEUTRAL_ZONE_FAR = CENTER + Units.inchesToMeters(120);
    public static final double OPP_HUB_CENTER =
        APTAG_FIELD_LAYOUT.getTagPose(4).get().getX() + Hub.WIDTH / 2.0;
    public static final double OPP_ALLIANCE_ZONE = APTAG_FIELD_LAYOUT.getTagPose(10).get().getX();
  }

  /**
   * Relevant field lines defined by Y-axis offset in meters.
   *
   * <p>Field element start/end labels are left-to-right from the perspective of that alliance
   * station.
   */
  public static class LinesHorizontal {

    public static final double CENTER = FIELD_WIDTH / 2.0;

    // Right of hub
    public static final double BLUE_RIGHT_BUMP_START = Hub.BLUE_NEAR_RIGHT_CORNER.getY();
    public static final double BLUE_RIGHT_BUMP_END = BLUE_RIGHT_BUMP_START - RightBump.WIDTH;
    public static final double BLUE_RIGHT_BUMP_MIDDLE =
        (BLUE_RIGHT_BUMP_START + BLUE_RIGHT_BUMP_END) / 2.0;
    public static final double BLUE_RIGHT_TRENCH_OPEN_START =
        BLUE_RIGHT_BUMP_END - Units.inchesToMeters(12.0);
    public static final double BLUE_RIGHT_TRENCH_OPEN_END = 0;

    public static final double RED_RIGHT_BUMP_END = Hub.RED_NEAR_RIGHT_CORNER.getY();
    public static final double RED_RIGHT_BUMP_START = RED_RIGHT_BUMP_END - RightBump.WIDTH;
    public static final double RED_RIGHT_BUMP_MIDDLE =
        (RED_RIGHT_BUMP_START + RED_RIGHT_BUMP_END) / 2.0;
    public static final double RED_RIGHT_TRENCH_OPEN_START =
        RED_RIGHT_BUMP_END - Units.inchesToMeters(12.0);
    public static final double RED_RIGHT_TRENCH_OPEN_END = 0;

    // Left of hub
    public static final double BLUE_LEFT_BUMP_END = Hub.BLUE_NEAR_LEFT_CORNER.getY();
    public static final double BLUE_LEFT_BUMP_START = BLUE_LEFT_BUMP_END + LeftBump.WIDTH;
    public static final double BLUE_LEFT_BUMP_MIDDLE =
        (BLUE_LEFT_BUMP_START + BLUE_LEFT_BUMP_END) / 2.0;
    public static final double BLUE_LEFT_TRENCH_OPEN_END =
        BLUE_LEFT_BUMP_START + Units.inchesToMeters(12.0);
    public static final double BLUE_LEFT_TRENCH_OPEN_START = FIELD_WIDTH;

    public static final double RED_LEFT_BUMP_START = Hub.RED_NEAR_LEFT_CORNER.getY();
    public static final double RED_LEFT_BUMP_END = RED_LEFT_BUMP_START + LeftBump.WIDTH;
    public static final double RED_LEFT_BUMP_MIDDLE =
        (RED_LEFT_BUMP_START + RED_LEFT_BUMP_END) / 2.0;
    public static final double RED_LEFT_TRENCH_OPEN_END =
        RED_LEFT_BUMP_END + Units.inchesToMeters(12.0);
    public static final double RED_LEFT_TRENCH_OPEN_START = FIELD_WIDTH;
  }

  /** Hub dimensions, corners, centers, and AprilTag face poses. */
  public static final class Hub {

    /* Dimensions */
    public static final double WIDTH = Units.inchesToMeters(47.0);
    public static final double HEIGHT =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double INNER_WIDTH = Units.inchesToMeters(41.7);
    public static final double INNER_HEIGHT = Units.inchesToMeters(56.5);

    // ── Blue Alliance ──────────────────────────────────────────────────────

    public static final Translation3d BLUE_TOP_CENTER_POINT =
        new Translation3d(
            APTAG_FIELD_LAYOUT.getTagPose(26).get().getX() + WIDTH / 2.0,
            FIELD_WIDTH / 2.0,
            HEIGHT);

    public static final Translation3d BLUE_INNER_CENTER_POINT =
        new Translation3d(
            APTAG_FIELD_LAYOUT.getTagPose(26).get().getX() + INNER_WIDTH / 2.0,
            FIELD_WIDTH / 2.0,
            INNER_HEIGHT);

    public static final Translation2d BLUE_CENTER_POSE =
        new Translation2d(
            APTAG_FIELD_LAYOUT.getTagPose(26).get().getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0);

    public static final Translation2d BLUE_NEAR_LEFT_CORNER =
        new Translation2d(
            BLUE_TOP_CENTER_POINT.getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0 + WIDTH / 2.0);
    public static final Translation2d BLUE_NEAR_RIGHT_CORNER =
        new Translation2d(
            BLUE_TOP_CENTER_POINT.getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0 - WIDTH / 2.0);
    public static final Translation2d BLUE_FAR_LEFT_CORNER =
        new Translation2d(
            BLUE_TOP_CENTER_POINT.getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0 + WIDTH / 2.0);
    public static final Translation2d BLUE_FAR_RIGHT_CORNER =
        new Translation2d(
            BLUE_TOP_CENTER_POINT.getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0 - WIDTH / 2.0);

    // Blue hub faces — sourced directly from AprilTag poses
    public static final Pose2d BLUE_NEAR_FACE = APTAG_FIELD_LAYOUT.getTagPose(26).get().toPose2d();
    public static final Pose2d BLUE_FAR_FACE = APTAG_FIELD_LAYOUT.getTagPose(20).get().toPose2d();
    public static final Pose2d BLUE_RIGHT_FACE = APTAG_FIELD_LAYOUT.getTagPose(18).get().toPose2d();
    public static final Pose2d BLUE_LEFT_FACE = APTAG_FIELD_LAYOUT.getTagPose(21).get().toPose2d();

    // ── Red Alliance ───────────────────────────────────────────────────────
    // Red hub is the field-symmetric mirror of blue:
    // X_red = FIELD_LENGTH - X_blue
    // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)

    public static final Translation3d RED_TOP_CENTER_POINT =
        new Translation3d(
            APTAG_FIELD_LAYOUT.getTagPose(10).get().getX() - WIDTH / 2.0,
            FIELD_WIDTH / 2.0,
            HEIGHT);

    public static final Translation3d RED_INNER_CENTER_POINT =
        new Translation3d(
            APTAG_FIELD_LAYOUT.getTagPose(10).get().getX() - INNER_WIDTH / 2.0,
            FIELD_WIDTH / 2.0,
            INNER_HEIGHT);

    public static final Translation2d RED_CENTER_POSE =
        new Translation2d(
            APTAG_FIELD_LAYOUT.getTagPose(10).get().getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0);

    public static final Translation2d RED_NEAR_LEFT_CORNER =
        new Translation2d(
            RED_TOP_CENTER_POINT.getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0 + WIDTH / 2.0);
    public static final Translation2d RED_NEAR_RIGHT_CORNER =
        new Translation2d(
            RED_TOP_CENTER_POINT.getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0 - WIDTH / 2.0);
    public static final Translation2d RED_FAR_LEFT_CORNER =
        new Translation2d(
            RED_TOP_CENTER_POINT.getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0 + WIDTH / 2.0);
    public static final Translation2d RED_FAR_RIGHT_CORNER =
        new Translation2d(
            RED_TOP_CENTER_POINT.getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0 - WIDTH / 2.0);

    // Red hub faces — sourced directly from AprilTag poses
    // Note: near/far and left/right are from the red driver station perspective
    public static final Pose2d RED_NEAR_FACE = APTAG_FIELD_LAYOUT.getTagPose(10).get().toPose2d();
    public static final Pose2d RED_FAR_FACE = APTAG_FIELD_LAYOUT.getTagPose(4).get().toPose2d();
    public static final Pose2d RED_RIGHT_FACE = APTAG_FIELD_LAYOUT.getTagPose(2).get().toPose2d();
    public static final Pose2d RED_LEFT_FACE = APTAG_FIELD_LAYOUT.getTagPose(5).get().toPose2d();
  }

  /** Left-side bump geometry, mirrored for each alliance perspective. */
  public static final class LeftBump {
    /* Dimensions */
    public static final double WIDTH = Units.inchesToMeters(73.0);
    public static final double HEIGHT = Units.inchesToMeters(6.513);
    public static final double DEPTH = Units.inchesToMeters(44.4);

    // ── Blue Alliance ──────────────────────────────────────────────────────
    public static final Translation2d BLUE_NEAR_LEFT_CORNER =
        Hub.BLUE_NEAR_LEFT_CORNER.plus(new Translation2d(0.0, WIDTH));
    public static final Translation2d BLUE_NEAR_RIGHT_CORNER =
        Hub.BLUE_NEAR_RIGHT_CORNER.plus(new Translation2d(0.0, WIDTH));
    public static final Translation2d BLUE_FAR_LEFT_CORNER =
        Hub.BLUE_FAR_LEFT_CORNER.plus(new Translation2d(0.0, WIDTH));
    public static final Translation2d BLUE_FAR_RIGHT_CORNER =
        Hub.BLUE_FAR_RIGHT_CORNER.plus(new Translation2d(0.0, WIDTH));

    // ── Red Alliance ───────────────────────────────────────────────────────
    // Red bump is the field-symmetric mirror of blue:
    // X_red = FIELD_LENGTH - X_blue
    // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)
    public static final Translation2d RED_NEAR_LEFT_CORNER =
        Hub.RED_NEAR_LEFT_CORNER.plus(new Translation2d(0.0, WIDTH));
    public static final Translation2d RED_NEAR_RIGHT_CORNER =
        Hub.RED_NEAR_RIGHT_CORNER.plus(new Translation2d(0.0, WIDTH));
    public static final Translation2d RED_FAR_LEFT_CORNER =
        Hub.RED_FAR_LEFT_CORNER.plus(new Translation2d(0.0, WIDTH));
    public static final Translation2d RED_FAR_RIGHT_CORNER =
        Hub.RED_FAR_RIGHT_CORNER.plus(new Translation2d(0.0, WIDTH));
  }

  /** Right-side bump geometry, mirrored for each alliance perspective. */
  public static final class RightBump {

    /* Dimensions */
    public static final double WIDTH = Units.inchesToMeters(73.0);
    public static final double HEIGHT = Units.inchesToMeters(6.513);
    public static final double DEPTH = Units.inchesToMeters(44.4);

    // ── Blue Alliance ──────────────────────────────────────────────────────
    public static final Translation2d BLUE_NEAR_LEFT_CORNER = Hub.BLUE_NEAR_RIGHT_CORNER;
    public static final Translation2d BLUE_NEAR_RIGHT_CORNER =
        Hub.BLUE_NEAR_RIGHT_CORNER.minus(new Translation2d(0.0, WIDTH));
    public static final Translation2d BLUE_FAR_LEFT_CORNER = Hub.BLUE_FAR_LEFT_CORNER;
    public static final Translation2d BLUE_FAR_RIGHT_CORNER =
        Hub.BLUE_FAR_RIGHT_CORNER.minus(new Translation2d(0.0, WIDTH));

    // ── Red Alliance ───────────────────────────────────────────────────────
    // Red bump is the field-symmetric mirror of blue:
    // X_red = FIELD_LENGTH - X_blue
    // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)
    public static final Translation2d RED_NEAR_LEFT_CORNER = Hub.RED_NEAR_RIGHT_CORNER;
    public static final Translation2d RED_NEAR_RIGHT_CORNER =
        Hub.RED_NEAR_RIGHT_CORNER.minus(new Translation2d(0.0, WIDTH));
    public static final Translation2d RED_FAR_LEFT_CORNER = Hub.RED_FAR_LEFT_CORNER;
    public static final Translation2d RED_FAR_RIGHT_CORNER =
        Hub.RED_FAR_RIGHT_CORNER.minus(new Translation2d(0.0, WIDTH));
  }

  /** Left trench dimensions and opening points. */
  public static final class LeftTrench {
    /* Dimensions */
    public static final double WIDTH = Units.inchesToMeters(65.65);
    public static final double DEPTH = Units.inchesToMeters(47.0);
    public static final double HEIGHT = Units.inchesToMeters(40.25);
    public static final double OPENING_WIDTH = Units.inchesToMeters(50.34);
    public static final double OPENING_HEIGHT = Units.inchesToMeters(22.25);

    // ── Blue Alliance ──────────────────────────────────────────────────────
    public static final Translation3d BLUE_OPENING_TOP_LEFT =
        new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH, OPENING_HEIGHT);
    public static final Translation3d BLUE_OPENING_TOP_RIGHT =
        new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH - OPENING_WIDTH, OPENING_HEIGHT);

    // ── Red Alliance ───────────────────────────────────────────────────────
    // Red trench is the field-symmetric mirror of blue:
    // X_red = FIELD_LENGTH - X_blue
    // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)
    public static final Translation3d RED_OPENING_TOP_LEFT =
        new Translation3d(LinesVertical.HUB_CENTER, 0.0, OPENING_HEIGHT);
    public static final Translation3d RED_OPENING_TOP_RIGHT =
        new Translation3d(LinesVertical.HUB_CENTER, OPENING_WIDTH, OPENING_HEIGHT);
  }

  /** Right trench dimensions and opening points. */
  public static final class RightTrench {
    /* Dimensions */
    public static final double WIDTH = Units.inchesToMeters(65.65);
    public static final double DEPTH = Units.inchesToMeters(47.0);
    public static final double HEIGHT = Units.inchesToMeters(40.25);
    public static final double OPENING_WIDTH = Units.inchesToMeters(50.34);
    public static final double OPENING_HEIGHT = Units.inchesToMeters(22.25);

    // ── Blue Alliance ──────────────────────────────────────────────────────
    public static final Translation3d BLUE_OPENING_TOP_LEFT =
        new Translation3d(LinesVertical.HUB_CENTER, OPENING_WIDTH, OPENING_HEIGHT);
    public static final Translation3d BLUE_OPENING_TOP_RIGHT =
        new Translation3d(LinesVertical.HUB_CENTER, 0, OPENING_HEIGHT);

    // ── Red Alliance ───────────────────────────────────────────────────────
    // Red trench is the field-symmetric mirror of blue:
    // X_red = FIELD_LENGTH - X_blue
    // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)
    public static final Translation3d RED_OPENING_TOP_LEFT =
        new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH - OPENING_WIDTH, OPENING_HEIGHT);
    public static final Translation3d RED_OPENING_TOP_RIGHT =
        new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH, OPENING_HEIGHT);
  }

  /** Depot dimensions and corner points. */
  public static final class Depot {
    /* Dimensions */
    public static final double WIDTH = Units.inchesToMeters(42.0);
    public static final double DEPTH = Units.inchesToMeters(27.0);
    public static final double HEIGHT = Units.inchesToMeters(1.125);
    public static final double DISTANCE_FROM_CENTER_Y = Units.inchesToMeters(75.93);

    // ── Blue Alliance ───────────────────────────────────────────────────────
    public static final Translation3d BLUE_DEPOT_CENTER =
        new Translation3d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y, HEIGHT);
    public static final Translation3d BLUE_LEFT_CORNER =
        new Translation3d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y + (WIDTH / 2), HEIGHT);
    public static final Translation3d BLUE_RIGHT_CORNER =
        new Translation3d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y - (WIDTH / 2), HEIGHT);

    // ── Red Alliance ───────────────────────────────────────────────────────
    // Red depot is the field-symmetric mirror of blue:
    // X_red = FIELD_LENGTH - X_blue
    // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)
    public static final Translation3d RED_DEPOT_CENTER =
        new Translation3d(FIELD_LENGTH - DEPTH, (FIELD_WIDTH / 2) - DISTANCE_FROM_CENTER_Y, HEIGHT);
    public static final Translation3d RED_LEFT_CORNER =
        new Translation3d(
            FIELD_LENGTH - DEPTH, (FIELD_WIDTH / 2) - DISTANCE_FROM_CENTER_Y + (WIDTH / 2), HEIGHT);
    public static final Translation3d RED_RIGHT_CORNER =
        new Translation3d(
            FIELD_LENGTH - DEPTH, (FIELD_WIDTH / 2) - DISTANCE_FROM_CENTER_Y - (WIDTH / 2), HEIGHT);
  }
}
