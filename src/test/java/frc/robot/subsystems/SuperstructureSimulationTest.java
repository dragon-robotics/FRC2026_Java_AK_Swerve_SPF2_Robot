package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure.Superstate;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.constants.FieldConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class SuperstructureSimulationTest {
  private static final double LOOP_PERIOD_SECONDS = 0.020;
  private static final int MAX_AIM_CYCLES = 700;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void stationaryAimedShotCoordinatesDriveAndFullMechanismPhysics() {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    double originalBatteryVoltage = RoboRioSim.getVInVoltage();
    Drive drive = null;
    Intake intake = null;
    Hopper hopper = null;
    Shooter shooter = null;
    Vision vision = null;
    Superstructure superstructure = null;

    try {
      SimHooks.pauseTiming();
      RoboRioSim.setVInVoltage(12.0);
      DriverStationSim.resetData();
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();

      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIOSim(TunerConstants.FrontLeft),
              new ModuleIOSim(TunerConstants.FrontRight),
              new ModuleIOSim(TunerConstants.BackLeft),
              new ModuleIOSim(TunerConstants.BackRight));
      intake = new Intake(new IntakeIOSim());
      hopper = new Hopper(new HopperIOSim());
      shooter = new Shooter(new ShooterIOSim());
      vision = new SuperstructureTestHarness.RecordingVision(drive);
      superstructure = new Superstructure(drive, intake, hopper, shooter, vision);

      Pose2d startPose =
          new Pose2d(1.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.fromDegrees(20.0));
      drive.setPoseAndSimulationTruth(startPose);
      superstructure.periodic();

      Command intakeCommand = superstructure.setStateCmd(Superstate.INTAKE);
      scheduler.schedule(intakeCommand);
      int intakeCycles = 0;
      while (intake.getCurrentState() != IntakeState.INTAKE && intakeCycles < 400) {
        runSimulationCycle(scheduler);
        intakeCycles++;
      }
      assertEquals(IntakeState.INTAKE, intake.getCurrentState());

      Command aimedShot = superstructure.selectedShootModeCmd();
      scheduler.schedule(aimedShot);
      int aimCycles = 0;
      while (!readyToFeed(intake, hopper, shooter, superstructure) && aimCycles < MAX_AIM_CYCLES) {
        runSimulationCycle(scheduler);
        aimCycles++;
      }

      assertEquals(IntakeState.INTAKE, intake.getCurrentState());
      assertEquals(ShooterState.SHOOT, shooter.getCurrentState());
      assertEquals(HopperState.INDEX_TO_SHOOTER, hopper.getDesiredState());
      assertTrue(superstructure.isAlignedToTarget());
      assertTrue(drive.getPose().getTranslation().getDistance(startPose.getTranslation()) < 0.25);
      assertTrue(aimCycles < MAX_AIM_CYCLES, "aimed shot did not converge before the cycle bound");
      System.out.printf(
          "Task 5 physics convergence: intake=%d cycles, aimed=%d cycles, heading=%.3f deg, drift=%.4f m%n",
          intakeCycles,
          aimCycles,
          drive.getPose().getRotation().getDegrees(),
          drive.getPose().getTranslation().getDistance(startPose.getTranslation()));
    } finally {
      scheduler.cancelAll();
      if (drive != null) scheduler.unregisterSubsystem(drive);
      if (intake != null) scheduler.unregisterSubsystem(intake);
      if (hopper != null) scheduler.unregisterSubsystem(hopper);
      if (shooter != null) scheduler.unregisterSubsystem(shooter);
      if (vision != null) scheduler.unregisterSubsystem(vision);
      if (superstructure != null) scheduler.unregisterSubsystem(superstructure);
      scheduler.getActiveButtonLoop().clear();
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
      RoboRioSim.setVInVoltage(originalBatteryVoltage);
      SimHooks.resumeTiming();
    }
  }

  private static void runSimulationCycle(CommandScheduler scheduler) {
    SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    DriverStationSim.notifyNewData();
    scheduler.run();
  }

  private static boolean readyToFeed(
      Intake intake, Hopper hopper, Shooter shooter, Superstructure superstructure) {
    return intake.getCurrentState() == IntakeState.INTAKE
        && shooter.getCurrentState() == ShooterState.SHOOT
        && hopper.getDesiredState() == HopperState.INDEX_TO_SHOOTER
        && superstructure.isAlignedToTarget();
  }
}
