package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionDriveBindings;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionRuntimeConfig;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;

public final class SuperstructureTestHarness implements AutoCloseable {
  public final MutableShooterIO shooterIO = new MutableShooterIO();
  public final Intake intake = new Intake(new IntakeIO.NoOp());
  public final Hopper hopper = new Hopper(new HopperIO.NoOp());
  public final Shooter shooter = new Shooter(shooterIO);
  public final RecordingDrive drive = new RecordingDrive();
  public final RecordingVision vision = new RecordingVision(drive);
  public final FixedTimer fixedTimer = new FixedTimer();
  public final Superstructure superstructure =
      new Superstructure(drive, intake, hopper, shooter, vision, () -> fixedTimer);

  public SuperstructureTestHarness() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  public void run(Command command) {
    command.schedule();
    runCycles(2);
  }

  public void runCycles(int count) {
    for (int cycle = 0; cycle < count; cycle++) {
      CommandScheduler.getInstance().run();
    }
  }

  public void setAlliance(AllianceStationID station) {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  public void setPose(Pose2d pose) {
    drive.setPoseAndSimulationTruth(pose);
    superstructure.periodic();
  }

  public void makeShooterReady() {
    shooterIO.flywheelRpm = shooter.getTargetRpm();
    shooterIO.hoodRotations = shooter.getTargetHoodRotations();
    shooter.periodic();
  }

  public Pose2d poseInside(FieldZones desiredZone) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    for (double x = 0.05; x < FieldConstants.FIELD_LENGTH; x += 0.05) {
      for (double y = 0.05; y < FieldConstants.FIELD_WIDTH; y += 0.05) {
        Pose2d candidate = new Pose2d(x, y, Rotation2d.kZero);
        if (FieldZones.fromPose(candidate, alliance) == desiredZone) {
          return candidate;
        }
      }
    }
    throw new IllegalArgumentException("No test pose found for " + desiredZone.name());
  }

  @Override
  public void close() {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    scheduler.unregisterAllSubsystems();
  }

  public static final class RecordingDrive extends Drive {
    private ChassisSpeeds lastRequestedSpeeds = new ChassisSpeeds();

    RecordingDrive() {
      super(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds) {
      lastRequestedSpeeds =
          new ChassisSpeeds(
              speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
      super.runVelocity(speeds);
    }

    public ChassisSpeeds lastRequestedSpeeds() {
      return lastRequestedSpeeds;
    }
  }

  public static final class RecordingVision extends Vision {
    public boolean aiming;
    public int reseedCalls;

    RecordingVision(Drive drive) {
      super(
          VisionDriveBindings.fromDrive(drive),
          VisionRuntimeConfig.fromSystemProperties(),
          () -> {},
          new VisionIO.NoOp("superstructure-test"));
    }

    @Override
    public void setAiming(boolean aiming) {
      this.aiming = aiming;
    }

    @Override
    public boolean forceReseedFromVision() {
      reseedCalls++;
      return true;
    }

    @Override
    public void periodic() {
      // Avoid mutating Vision startup-strategy globals in Superstructure-only tests.
    }
  }

  public static final class MutableShooterIO implements ShooterIO {
    public double flywheelRpm;
    public double hoodRotations;
    public double kickerDutyCycle;
    public double kickerVoltage;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      inputs.flywheelLeadVelocityRpm = flywheelRpm;
      inputs.hoodPositionRotations = hoodRotations;
    }

    @Override
    public void setKickerDutyCycle(double output) {
      kickerDutyCycle = output;
    }

    @Override
    public void setKickerVoltage(Voltage voltage) {
      kickerVoltage = voltage.in(Volts);
    }
  }

  public static final class FixedTimer extends Timer {
    private double time;

    public void setTime(double time) {
      this.time = time;
    }

    @Override
    public double get() {
      return time;
    }

    @Override
    public void restart() {
      time = 0.0;
    }
  }
}
