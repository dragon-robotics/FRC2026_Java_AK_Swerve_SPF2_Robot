package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.vision.Vision;
import java.util.Objects;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
  /** Robot Superstates */
  public enum Superstate {
    DRIVE_STARTING_CONFIG,
    DRIVE,
    INTAKE,
    OUTTAKE,
    SHOOT_WITH_AIM,
    SHOOT_NO_AIM,
    MANUAL_SHOOT,
    PURGE
  }

  public enum ShootMode {
    DEFAULT_SHOOT_WITH_AIM,
    MANUAL_BUMPER_UP,
    MANUAL_TRENCH
  }

  private final Drive drive;
  private final Intake intake;
  private final Hopper hopper;
  private final Shooter shooter;
  private final Vision vision;
  private final Supplier<Timer> timerFactory;

  private Superstate currentState = Superstate.DRIVE_STARTING_CONFIG;
  private ShootMode shootMode = ShootMode.DEFAULT_SHOOT_WITH_AIM;

  public Superstructure(Drive drive, Intake intake, Hopper hopper, Shooter shooter, Vision vision) {
    this(drive, intake, hopper, shooter, vision, Timer::new);
  }

  Superstructure(
      Drive drive,
      Intake intake,
      Hopper hopper,
      Shooter shooter,
      Vision vision,
      Supplier<Timer> timerFactory) {
    this.drive = Objects.requireNonNull(drive);
    this.intake = Objects.requireNonNull(intake);
    this.hopper = Objects.requireNonNull(hopper);
    this.shooter = Objects.requireNonNull(shooter);
    this.vision = Objects.requireNonNull(vision);
    this.timerFactory = Objects.requireNonNull(timerFactory);
    configureSafeDefaults();
  }

  public Superstate getCurrentState() {
    return currentState;
  }

  public ShootMode getShootMode() {
    return shootMode;
  }

  private void configureSafeDefaults() {
    intake.setDesiredState(IntakeState.HOME);
    hopper.setDesiredState(HopperState.STOP);
    shooter.setDesiredState(ShooterState.PREPFUEL);

    intake.setDefaultCommand(Commands.idle(intake).withName("Intake.Default(HOLD_LAST_REQUEST)"));
    hopper.setDefaultCommand(
        Commands.run(() -> hopper.setDesiredState(HopperState.STOP), hopper)
            .withName("Hopper.Default(STOP)"));
    shooter.setDefaultCommand(
        Commands.run(() -> shooter.setDesiredState(ShooterState.PREPFUEL), shooter)
            .withName("Shooter.Default(PREPFUEL)"));
  }

  public Command setStateCmd(Superstate desiredState) {
    Objects.requireNonNull(desiredState);
    return switch (desiredState) {
      case DRIVE_STARTING_CONFIG -> mechanismStateCommand(
          desiredState, IntakeState.HOME, HopperState.STOP, ShooterState.PREPFUEL);
      case DRIVE -> mechanismStateCommand(
          desiredState, IntakeState.DEPLOYED, HopperState.STOP, ShooterState.PREPFUEL);
      case INTAKE -> mechanismStateCommand(
          desiredState, IntakeState.INTAKE, HopperState.STOP, ShooterState.PREPFUEL);
      case OUTTAKE -> mechanismStateCommand(
          desiredState, IntakeState.OUTTAKE, HopperState.INDEX_TO_INTAKE, ShooterState.PREPFUEL);
      case SHOOT_WITH_AIM, SHOOT_NO_AIM, MANUAL_SHOOT -> shootingPreparationCommand(desiredState);
      case PURGE -> purgePreparationCommand();
    };
  }

  public Command intakeOverrideCmd(IntakeState state) {
    IntakeState requestedState = Objects.requireNonNull(state);
    return Commands.run(() -> intake.setDesiredState(requestedState), intake)
        .withName("Intake.Override(" + requestedState.name() + ")");
  }

  public Command hopperOverrideCmd(HopperState state) {
    HopperState requestedState = Objects.requireNonNull(state);
    return Commands.run(() -> hopper.setDesiredState(requestedState), hopper)
        .withName("Hopper.Override(" + requestedState.name() + ")");
  }

  public Command shooterOverrideCmd(ShooterState state) {
    ShooterState requestedState = Objects.requireNonNull(state);
    return Commands.run(() -> shooter.setDesiredState(requestedState), shooter)
        .withName("Shooter.Override(" + requestedState.name() + ")");
  }

  private Command mechanismStateCommand(
      Superstate superstate,
      IntakeState intakeState,
      HopperState hopperState,
      ShooterState shooterState) {
    return Commands.run(
            () -> {
              intake.setDesiredState(intakeState);
              hopper.setDesiredState(hopperState);
              shooter.setDesiredState(shooterState);
              currentState = superstate;
            },
            this,
            intake,
            hopper,
            shooter)
        .withName("Superstate(" + superstate.name() + ")");
  }

  private Command shootingPreparationCommand(Superstate superstate) {
    return Commands.run(
            () -> {
              hopper.setDesiredState(HopperState.STOP);
              shooter.setDesiredState(ShooterState.SHOOT);
              currentState = superstate;
            },
            this,
            hopper,
            shooter)
        .withName("Superstate(" + superstate.name() + ":PREPARING)");
  }

  private Command purgePreparationCommand() {
    return Commands.run(
            () -> {
              intake.setDesiredState(IntakeState.OUTTAKE);
              hopper.setDesiredState(HopperState.STOP);
              shooter.setDesiredState(ShooterState.SHOOT);
              currentState = Superstate.PURGE;
            },
            this,
            intake,
            hopper,
            shooter)
        .withName("Superstate(PURGE:PREPARING)");
  }
}
