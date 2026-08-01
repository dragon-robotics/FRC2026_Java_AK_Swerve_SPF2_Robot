package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.util.constants.FieldConstants.FieldZones;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class DriveCommandsTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void processJoystickInputsAppliesDeadbandSquaringAndHalfSpeed() {
    var full = DriveCommands.processJoystickInputs(0.5, -0.5, 0.5, false, 10.0, 4.0);
    var half = DriveCommands.processJoystickInputs(0.5, -0.5, 0.5, true, 10.0, 4.0);

    assertEquals(160.0 / 81.0, full.translation(), 1e-9);
    assertEquals(-160.0 / 81.0, full.strafe(), 1e-9);
    assertEquals(64.0 / 81.0, full.rotation(), 1e-9);
    assertEquals(8.0 / 9.0, half.translation(), 1e-9);
    assertEquals(-8.0 / 9.0, half.strafe(), 1e-9);
    assertEquals(16.0 / 45.0, half.rotation(), 1e-9);
  }

  @Test
  void processJoystickInputsZerosValuesInsideDeadband() {
    var speeds = DriveCommands.processJoystickInputs(0.09, -0.09, 0.09, false, 10.0, 4.0);

    assertEquals(0.0, speeds.translation(), 1e-9);
    assertEquals(0.0, speeds.strafe(), 1e-9);
    assertEquals(0.0, speeds.rotation(), 1e-9);
  }

  @Test
  void selectDriveModeGivesZoneLockPrecedence() {
    assertEquals(
        DriveCommands.DriveMode.LOCK_ZONE, DriveCommands.selectDriveMode(true, true, true, true));
    assertEquals(
        DriveCommands.DriveMode.ROTATE, DriveCommands.selectDriveMode(false, true, true, true));
    assertEquals(
        DriveCommands.DriveMode.HOLD_HEADING,
        DriveCommands.selectDriveMode(false, false, false, false));
  }

  @Test
  void isRotationActiveRequiresRecentTriggerAndMeasuredMotion() {
    assertTrue(DriveCommands.isRotationActive(10.0, 10.05, Math.toRadians(11.0)));
    assertFalse(DriveCommands.isRotationActive(10.0, 10.11, Math.toRadians(11.0)));
    assertFalse(DriveCommands.isRotationActive(10.0, 10.05, Math.toRadians(10.0)));
  }

  @Test
  void closedLoopEntryResetsControllerWhenHeadingWasClearedByManualRotation() {
    assertTrue(
        DriveCommands.shouldResetOrientationController(
            DriveCommands.DriveMode.HOLD_HEADING, Optional.empty()));
    assertTrue(
        DriveCommands.shouldResetOrientationController(
            DriveCommands.DriveMode.LOCK_ZONE, Optional.empty()));
    assertFalse(
        DriveCommands.shouldResetOrientationController(
            DriveCommands.DriveMode.HOLD_HEADING, Optional.of(Rotation2d.kZero)));
    assertFalse(
        DriveCommands.shouldResetOrientationController(
            DriveCommands.DriveMode.ROTATE, Optional.empty()));
  }

  @Test
  void toFieldRelativeSpeedsFlipsRobotHeadingForRedAlliance() {
    var blue =
        DriveCommands.toFieldRelativeSpeeds(
            new ChassisSpeeds(1.0, 0.0, 0.0), Rotation2d.kZero, Alliance.Blue);
    var red =
        DriveCommands.toFieldRelativeSpeeds(
            new ChassisSpeeds(1.0, 0.0, 0.0), Rotation2d.kZero, Alliance.Red);

    assertEquals(1.0, blue.vxMetersPerSecond, 1e-9);
    assertEquals(-1.0, red.vxMetersPerSecond, 1e-9);
  }

  @Test
  void getZoneLockedHeadingMapsSupportedZonesForEachAlliance() {
    assertEquals(
        -45.0,
        DriveCommands.getZoneLockedHeading(FieldZones.ALLIANCE_LEFT, Alliance.Blue)
            .orElseThrow()
            .getDegrees(),
        1e-9);
    assertEquals(
        45.0,
        DriveCommands.getZoneLockedHeading(FieldZones.NEUTRAL_RIGHT, Alliance.Blue)
            .orElseThrow()
            .getDegrees(),
        1e-9);
    assertEquals(
        135.0,
        DriveCommands.getZoneLockedHeading(FieldZones.OPPONENT_LEFT, Alliance.Red)
            .orElseThrow()
            .getDegrees(),
        1e-9);
    assertEquals(
        -135.0,
        DriveCommands.getZoneLockedHeading(FieldZones.NEUTRAL_RIGHT_PURGE, Alliance.Red)
            .orElseThrow()
            .getDegrees(),
        1e-9);
  }

  @Test
  void getZoneLockedHeadingExcludesTrenchAndBumpZones() {
    assertTrue(
        DriveCommands.getZoneLockedHeading(FieldZones.ALLIANCE_LEFT_TRENCH, Alliance.Blue)
            .isEmpty());
    assertTrue(
        DriveCommands.getZoneLockedHeading(FieldZones.OPPONENT_RIGHT_BUMP, Alliance.Red).isEmpty());
  }

  @Test
  void joystickDriveAtAnglePublishesTheActiveTargetHeading() {
    AtomicReference<Optional<Rotation2d>> observed = new AtomicReference<>(Optional.empty());
    Rotation2d target = Rotation2d.fromDegrees(37.0);
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    try {
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      Command command =
          DriveCommands.joystickDriveAtAngle(
              drive, () -> 0.0, () -> 0.0, () -> target, observed::set);
      CommandScheduler.getInstance().schedule(command);
      CommandScheduler.getInstance().run();
      CommandScheduler.getInstance().run();
      assertEquals(Optional.of(target), observed.get());
    } finally {
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().unregisterSubsystem(drive);
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    }
  }
}
