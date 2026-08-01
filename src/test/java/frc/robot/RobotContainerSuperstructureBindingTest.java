package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure.ShootMode;
import frc.robot.subsystems.Superstructure.Superstate;
import frc.robot.subsystems.SuperstructureTestHarness;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.util.constants.FieldConstants.FieldZones;
import java.util.Optional;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RobotContainerSuperstructureBindingTest {
  private static final int DRIVER_PORT = 4;
  private static final int OPERATOR_PORT = 5;

  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private SuperstructureTestHarness harness;
  private XboxControllerSim driverSim;
  private XboxControllerSim operatorSim;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    scheduler.cancelAll();
    scheduler.unregisterAllSubsystems();
    scheduler.getActiveButtonLoop().clear();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    harness = new SuperstructureTestHarness();
    CommandXboxController driver = new CommandXboxController(DRIVER_PORT);
    CommandXboxController operator = new CommandXboxController(OPERATOR_PORT);
    driverSim = new XboxControllerSim(DRIVER_PORT);
    operatorSim = new XboxControllerSim(OPERATOR_PORT);
    configureXboxSimulation(driverSim);
    configureXboxSimulation(operatorSim);
    RobotContainer.configureSuperstructureBindings(
        driver,
        operator,
        harness.drive,
        harness.intake,
        harness.hopper,
        harness.shooter,
        harness.superstructure);
  }

  @AfterEach
  void tearDown() {
    harness.close();
    scheduler.getActiveButtonLoop().clear();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  @Test
  void driverAUsesSafeConfigWhileHeldAndDriveOnRelease() {
    setDriverButton(XboxController.Button.kA, true);
    assertEquals(Superstate.DRIVE_STARTING_CONFIG, harness.superstructure.getCurrentState());
    assertEquals(IntakeState.HOME, harness.intake.getDesiredState());

    setDriverButton(XboxController.Button.kA, false);
    assertEquals(Superstate.DRIVE, harness.superstructure.getCurrentState());
    assertEquals(IntakeState.DEPLOYED, harness.intake.getDesiredState());
  }

  @Test
  void mechanismButtonsRestoreDriveOrDeployedOnRelease() {
    setDriverAxis(XboxController.Axis.kLeftTrigger, 1.0);
    assertEquals(IntakeState.INTAKE, harness.intake.getDesiredState());

    setDriverAxis(XboxController.Axis.kLeftTrigger, 0.0);
    assertEquals(Superstate.DRIVE, harness.superstructure.getCurrentState());

    setDriverButton(XboxController.Button.kRightBumper, true);
    assertEquals(Superstate.OUTTAKE, harness.superstructure.getCurrentState());
    assertEquals(IntakeState.OUTTAKE, harness.intake.getDesiredState());

    setDriverButton(XboxController.Button.kRightBumper, false);
    assertEquals(Superstate.DRIVE, harness.superstructure.getCurrentState());

    setDriverButton(XboxController.Button.kB, true);
    assertEquals(IntakeState.JUICER, harness.intake.getDesiredState());

    setDriverButton(XboxController.Button.kB, false);
    assertEquals(IntakeState.DEPLOYED, harness.intake.getDesiredState());
  }

  @Test
  void rightTriggerDefersPurgeRoutingAndAlwaysRestoresDrive() {
    harness.setAlliance(AllianceStationID.Blue1);
    harness.setPose(harness.poseInside(FieldZones.NEUTRAL_LEFT_PURGE));

    setDriverAxis(XboxController.Axis.kRightTrigger, 1.0);

    assertEquals(Superstate.PURGE, harness.superstructure.getCurrentState());
    assertEquals(IntakeState.OUTTAKE, harness.intake.getDesiredState());
    assertEquals(
        Set.of(
            harness.superstructure, harness.drive, harness.intake, harness.hopper, harness.shooter),
        requiring(harness.superstructure).getRequirements());

    setDriverAxis(XboxController.Axis.kRightTrigger, 0.0);
    assertEquals(Superstate.DRIVE, harness.superstructure.getCurrentState());
  }

  @Test
  void rightTriggerDefersSelectedManualModeWithoutTakingIntake() {
    harness.run(harness.superstructure.setShootModeCmd(ShootMode.MANUAL_BUMPER_UP));

    setDriverAxis(XboxController.Axis.kRightTrigger, 1.0);

    assertEquals(Superstate.MANUAL_SHOOT, harness.superstructure.getCurrentState());
    assertEquals(
        Set.of(harness.superstructure, harness.drive, harness.hopper, harness.shooter),
        requiring(harness.superstructure).getRequirements());

    setDriverAxis(XboxController.Axis.kRightTrigger, 0.0);
    assertEquals(Superstate.DRIVE, harness.superstructure.getCurrentState());
  }

  @Test
  void rightTriggerRejectsDisallowedDefaultShotAndRestoresDriveOnRelease() {
    harness.setAlliance(AllianceStationID.Blue1);
    harness.setPose(harness.poseInside(FieldZones.ALLIANCE_LEFT_BUMP));

    setDriverAxis(XboxController.Axis.kRightTrigger, 1.0);
    assertEquals(Superstate.DRIVE_STARTING_CONFIG, harness.superstructure.getCurrentState());
    assertNull(scheduler.requiring(harness.superstructure));

    setDriverAxis(XboxController.Axis.kRightTrigger, 0.0);
    assertEquals(Superstate.DRIVE, harness.superstructure.getCurrentState());
  }

  @Test
  void driverHeadingResetAlsoClearsHeldHeading() {
    Pose2d beforeReset = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(70.0));
    harness.drive.setPose(beforeReset);
    harness.superstructure.setCurrentHeading(Optional.of(Rotation2d.fromDegrees(35.0)));

    driverSim.setRawButton(XboxController.Button.kStart.value, true);
    driverSim.setRawButton(XboxController.Button.kBack.value, true);
    runScheduler();

    assertEquals(beforeReset.getTranslation(), harness.drive.getPose().getTranslation());
    assertEquals(Rotation2d.kZero, harness.drive.getPose().getRotation());
    assertTrue(harness.superstructure.getCurrentHeading().isEmpty());
  }

  @Test
  void operatorReseedAndJuicerBindingsUseApprovedChords() {
    operatorSim.setRawButton(XboxController.Button.kStart.value, true);
    operatorSim.setRawButton(XboxController.Button.kBack.value, true);
    runScheduler();
    assertEquals(1, harness.vision.reseedCalls);

    operatorSim.setRawButton(XboxController.Button.kStart.value, false);
    operatorSim.setRawButton(XboxController.Button.kBack.value, false);
    runScheduler();

    setOperatorButton(XboxController.Button.kB, true);
    assertEquals(IntakeState.JUICER, harness.intake.getDesiredState());

    setOperatorButton(XboxController.Button.kB, false);
    assertEquals(IntakeState.DEPLOYED, harness.intake.getDesiredState());
  }

  @Test
  void operatorChordsToggleBothManualShootModes() {
    operatorSim.setRawButton(XboxController.Button.kA.value, true);
    operatorSim.setRawButton(XboxController.Button.kX.value, true);
    runScheduler();
    assertEquals(ShootMode.MANUAL_BUMPER_UP, harness.superstructure.getShootMode());

    operatorSim.setRawButton(XboxController.Button.kA.value, false);
    operatorSim.setRawButton(XboxController.Button.kX.value, false);
    runScheduler();

    operatorSim.setRawButton(XboxController.Button.kA.value, true);
    operatorSim.setRawButton(XboxController.Button.kB.value, true);
    runScheduler();
    assertEquals(ShootMode.MANUAL_TRENCH, harness.superstructure.getShootMode());
    assertEquals(
        IntakeState.HOME,
        harness.intake.getDesiredState(),
        "A+B must not also activate operator Juicer");
  }

  @Test
  void operatorRightBumperOverridesKickerAndStateMachineResumesAfterRelease() {
    runScheduler();
    setOperatorButton(XboxController.Button.kRightBumper, true);
    assertEquals(1.0, harness.shooterIO.kickerDutyCycle, 1e-9);
    assertEquals(Set.of(harness.shooter), requiring(harness.shooter).getRequirements());

    setOperatorButton(XboxController.Button.kRightBumper, false);
    assertEquals(0.0, harness.shooterIO.kickerDutyCycle, 1e-9);

    harness.shooterIO.kickerVoltage = 0.0;
    runScheduler();
    assertEquals(6.0, harness.shooterIO.kickerVoltage, 1e-9);
  }

  private Command requiring(edu.wpi.first.wpilibj2.command.Subsystem subsystem) {
    Command command = scheduler.requiring(subsystem);
    if (command == null) {
      throw new AssertionError("No command requires " + subsystem.getName());
    }
    return command;
  }

  private static void configureXboxSimulation(XboxControllerSim controller) {
    controller.setAxisCount(6);
    controller.setButtonCount(10);
    controller.setPOVCount(1);
  }

  private void setDriverButton(XboxController.Button button, boolean pressed) {
    driverSim.setRawButton(button.value, pressed);
    runScheduler();
  }

  private void setOperatorButton(XboxController.Button button, boolean pressed) {
    operatorSim.setRawButton(button.value, pressed);
    runScheduler();
  }

  private void setDriverAxis(XboxController.Axis axis, double value) {
    driverSim.setRawAxis(axis.value, value);
    runScheduler();
  }

  private void runScheduler() {
    DriverStationSim.notifyNewData();
    scheduler.run();
  }
}
