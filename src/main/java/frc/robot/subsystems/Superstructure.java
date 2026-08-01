package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.HubShiftUtil.ShiftEnum;
import frc.robot.util.HubShiftUtil.ShiftInfo;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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

  private static final double ALIGNMENT_TOLERANCE_DEGREES = 5.0;
  private static final double NEUTRAL_ZONE_HOOD_ROTATIONS = 1.25;
  private static final double MANUAL_BUMPER_UP_RPM = 2500.0;
  private static final double MANUAL_BUMPER_UP_HOOD_ROTATIONS = 0.0;
  private static final double MANUAL_TRENCH_RPM = 2900.0;
  private static final double MANUAL_TRENCH_HOOD_ROTATIONS = 0.75;

  private final Drive drive;
  private final Intake intake;
  private final Hopper hopper;
  private final Shooter shooter;
  private final Vision vision;
  private final Supplier<Timer> timerFactory;

  private Superstate currentState = Superstate.DRIVE_STARTING_CONFIG;
  private ShootMode shootMode = ShootMode.DEFAULT_SHOOT_WITH_AIM;
  private Alliance alliance = Alliance.Blue;
  private boolean allianceConfirmed;
  private FieldZones currentZone;
  private Translation2d currentAimTarget = FieldConstants.Hub.BLUE_CENTER_POSE;
  private double distanceToTargetMeters;
  private boolean alignedToTarget;
  private Optional<Rotation2d> currentHeading = Optional.empty();
  private double rotationLastTriggered;
  private ShiftInfo shiftedShiftInfo = new ShiftInfo(ShiftEnum.DISABLED, 0.0, 0.0, false);

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

  public boolean isAlignedToTarget() {
    return alignedToTarget;
  }

  public boolean isShootAllowed() {
    return SuperstructureTargeting.isShootAllowed(allianceConfirmed, currentZone);
  }

  public boolean shouldUsePurgeDuringShoot() {
    return shootMode == ShootMode.DEFAULT_SHOOT_WITH_AIM
        && SuperstructureTargeting.isPurgeZone(allianceConfirmed, currentZone);
  }

  public boolean isSelectedShootAllowed() {
    return shootMode != ShootMode.DEFAULT_SHOOT_WITH_AIM
        || SuperstructureTargeting.isShootAllowed(allianceConfirmed, currentZone);
  }

  public boolean isHubActive() {
    return shiftedShiftInfo.active();
  }

  public double getShiftTimeRemaining() {
    return shiftedShiftInfo.remainingTime();
  }

  public Optional<Rotation2d> getCurrentHeading() {
    return currentHeading;
  }

  public void setCurrentHeading(Optional<Rotation2d> heading) {
    currentHeading = Objects.requireNonNull(heading);
  }

  public double getRotationLastTriggered() {
    return rotationLastTriggered;
  }

  public void setRotationLastTriggered(double timestampSeconds) {
    rotationLastTriggered = timestampSeconds;
  }

  public Optional<Rotation2d> getZoneLockedHeading() {
    if (!allianceConfirmed || currentZone == null) return Optional.empty();
    return DriveCommands.getZoneLockedHeading(currentZone, alliance);
  }

  Translation2d getCurrentAimTarget() {
    return currentAimTarget;
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
      case SHOOT_WITH_AIM -> shootWithAimCmd();
      case SHOOT_NO_AIM -> shootNoAimCmd();
      case MANUAL_SHOOT -> manualShootCmd();
      case PURGE -> purgeCmd();
    };
  }

  public Command selectedShootModeCmd() {
    return switch (shootMode) {
      case DEFAULT_SHOOT_WITH_AIM -> shootWithAimCmd();
      case MANUAL_BUMPER_UP -> createManualShootStateCommand(
              MANUAL_BUMPER_UP_RPM, MANUAL_BUMPER_UP_HOOD_ROTATIONS)
          .withName("Superstate(SHOOT->MANUAL_BUMPER_UP)");
      case MANUAL_TRENCH -> createManualShootStateCommand(
              MANUAL_TRENCH_RPM, MANUAL_TRENCH_HOOD_ROTATIONS)
          .withName("Superstate(SHOOT->MANUAL_TRENCH)");
    };
  }

  public Command purgeShootCmd() {
    return purgeCmd().withName("Superstate(SHOOT->PURGE)");
  }

  public Command forceReseedFromVisionCmd() {
    return Commands.runOnce(vision::forceReseedFromVision, vision).withName("Force Vision Reseed");
  }

  public Command shootNoAimWithJuicerDelayCmd() {
    Timer timer = Objects.requireNonNull(timerFactory.get());
    return Commands.runOnce(timer::restart)
        .andThen(
            Commands.run(
                () -> {
                  currentState = Superstate.SHOOT_NO_AIM;
                  intake.setDesiredState(
                      timer.hasElapsed(1.5) ? IntakeState.JUICER : IntakeState.DEPLOYED);
                  shooter.setDesiredState(ShooterState.SHOOT);
                  setHopperFeedWhenReady(true);
                },
                this,
                intake,
                shooter,
                hopper))
        .withName("Superstate(SHOOT_NO_AIM+JUICER)");
  }

  public Command shootWithJuicerDelayCmd() {
    Timer timer = Objects.requireNonNull(timerFactory.get());
    return Commands.runOnce(timer::restart)
        .andThen(
            Commands.run(
                    () -> {
                      boolean purge =
                          SuperstructureTargeting.isPurgeZone(allianceConfirmed, currentZone);
                      currentState = purge ? Superstate.PURGE : Superstate.SHOOT_WITH_AIM;
                      intake.setDesiredState(
                          purge
                              ? IntakeState.OUTTAKE
                              : timer.hasElapsed(1.5) ? IntakeState.JUICER : IntakeState.DEPLOYED);
                      shooter.setDesiredState(ShooterState.SHOOT);
                      setHopperFeedWhenReady(true);
                    },
                    this,
                    intake,
                    shooter,
                    hopper)
                .alongWith(stationaryAimCommand()))
        .withName("Superstate(SHOOT_WITH_AIM+JUICER)");
  }

  public Command setShootModeCmd(ShootMode mode) {
    Objects.requireNonNull(mode);
    return Commands.runOnce(() -> shootMode = mode).withName("SetShootMode(" + mode.name() + ")");
  }

  public Command toggleShootModeCmd(ShootMode manualMode) {
    Objects.requireNonNull(manualMode);
    return Commands.runOnce(
            () ->
                shootMode = shootMode == manualMode ? ShootMode.DEFAULT_SHOOT_WITH_AIM : manualMode)
        .withName("ToggleShootMode(" + manualMode.name() + ")");
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

  private void setHopperFeedWhenReady(boolean requireAlignment) {
    boolean shooterReady = shooter.getCurrentState() == ShooterState.SHOOT;
    boolean alignmentReady = !requireAlignment || alignedToTarget;
    hopper.setDesiredState(
        shooterReady && alignmentReady ? HopperState.INDEX_TO_SHOOTER : HopperState.STOP);
  }

  private Command createShootMechanismCommand(Superstate state, boolean requireAlignment) {
    return Commands.run(
        () -> {
          shooter.setDesiredState(ShooterState.SHOOT);
          setHopperFeedWhenReady(requireAlignment);
          currentState = state;
        },
        this,
        shooter,
        hopper);
  }

  private Command stationaryAimCommand() {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        () -> 0.0,
        () -> 0.0,
        () -> SuperstructureTargeting.geometricTargetHeading(drive.getPose(), currentAimTarget),
        this::setCurrentHeading);
  }

  private Command shootWithAimCmd() {
    if (!isShootAllowed()) {
      return Commands.idle().withName("Superstate(SHOOT_WITH_AIM:DISALLOWED)");
    }
    if (SuperstructureTargeting.isPurgeZone(allianceConfirmed, currentZone)) {
      return purgeCmd();
    }
    return createShootMechanismCommand(Superstate.SHOOT_WITH_AIM, true)
        .alongWith(stationaryAimCommand())
        .withName("Superstate(SHOOT_WITH_AIM)");
  }

  private Command shootNoAimCmd() {
    if (!isShootAllowed()) {
      return Commands.idle().withName("Superstate(SHOOT_NO_AIM:DISALLOWED)");
    }
    return createShootMechanismCommand(Superstate.SHOOT_NO_AIM, true)
        .withName("Superstate(SHOOT_NO_AIM)");
  }

  private Command createManualShootStateCommand(double rpm, double hoodRotations) {
    return Commands.run(
            () -> {
              shooter.setSetpoint(rpm, hoodRotations);
              shooter.setDesiredState(ShooterState.SHOOT);
              setHopperFeedWhenReady(false);
              currentState = Superstate.MANUAL_SHOOT;
            },
            this,
            shooter,
            hopper)
        .alongWith(Commands.run(drive::stopWithX, drive));
  }

  private Command manualShootCmd() {
    return createManualShootStateCommand(shooter.getTargetRpm(), shooter.getTargetHoodRotations())
        .withName("Superstate(MANUAL_SHOOT)");
  }

  private Command createPurgeMechanismCommand() {
    return Commands.run(
        () -> {
          intake.setDesiredState(IntakeState.OUTTAKE);
          shooter.setDesiredState(ShooterState.SHOOT);
          setHopperFeedWhenReady(true);
          currentState = Superstate.PURGE;
        },
        this,
        intake,
        shooter,
        hopper);
  }

  private Command purgeCmd() {
    if (!SuperstructureTargeting.isPurgeZone(allianceConfirmed, currentZone)) {
      return Commands.idle().withName("Superstate(PURGE:DISALLOWED)");
    }
    return createPurgeMechanismCommand()
        .alongWith(stationaryAimCommand())
        .withName("Superstate(PURGE)");
  }

  private void updateTargeting(Pose2d pose) {
    Optional<Alliance> dsAlliance = DriverStation.getAlliance();
    if (dsAlliance.isPresent()) {
      alliance = dsAlliance.orElseThrow();
      allianceConfirmed = true;
      currentZone = FieldZones.fromPose(pose, alliance);
    } else {
      alliance = Alliance.Blue;
      allianceConfirmed = false;
      currentZone = null;
    }

    currentAimTarget =
        SuperstructureTargeting.resolveAimTarget(allianceConfirmed, currentZone, alliance);
    distanceToTargetMeters = pose.getTranslation().getDistance(currentAimTarget);
    alignedToTarget =
        SuperstructureTargeting.isAligned(pose, currentAimTarget, ALIGNMENT_TOLERANCE_DEGREES);

    if (currentState != Superstate.MANUAL_SHOOT) {
      if (SuperstructureTargeting.isNeutralShootOrPurgeZone(allianceConfirmed, currentZone)) {
        shooter.setSetpointForDistance(distanceToTargetMeters, NEUTRAL_ZONE_HOOD_ROTATIONS);
      } else {
        shooter.setSetpointForDistance(distanceToTargetMeters);
      }
    }
  }

  private boolean isFeedReadyForCurrentState() {
    boolean shooterReady = shooter.getCurrentState() == ShooterState.SHOOT;
    return switch (currentState) {
      case SHOOT_WITH_AIM, SHOOT_NO_AIM, PURGE -> shooterReady && alignedToTarget;
      case MANUAL_SHOOT -> shooterReady;
      default -> false;
    };
  }

  private static void recordShiftInfo(String prefix, ShiftInfo shiftInfo) {
    Logger.recordOutput(prefix + "/CurrentShift", shiftInfo.currentShift().name());
    Logger.recordOutput(prefix + "/Active", shiftInfo.active());
    Logger.recordOutput(prefix + "/ElapsedTime", shiftInfo.elapsedTime());
    Logger.recordOutput(prefix + "/RemainingTime", shiftInfo.remainingTime());
  }

  @Override
  public void periodic() {
    updateTargeting(drive.getPose());
    vision.setAiming(
        currentState == Superstate.SHOOT_WITH_AIM || currentState == Superstate.SHOOT_NO_AIM);

    ShiftInfo officialShiftInfo = HubShiftUtil.getOfficialShiftInfo();
    shiftedShiftInfo = HubShiftUtil.getShiftedShiftInfo();

    Logger.recordOutput("Superstructure/CurrentState", currentState.name());
    Logger.recordOutput("Superstructure/ShootMode", shootMode.name());
    Logger.recordOutput("Superstructure/AllianceConfirmed", allianceConfirmed);
    Logger.recordOutput(
        "Superstructure/Alliance", allianceConfirmed ? alliance.name() : "UNCONFIRMED");
    Logger.recordOutput(
        "Superstructure/Zone",
        allianceConfirmed && currentZone != null ? currentZone.name() : "UNCONFIRMED");
    Logger.recordOutput("Superstructure/AimTarget", currentAimTarget);
    Logger.recordOutput("Superstructure/DistanceToTargetMeters", distanceToTargetMeters);
    Logger.recordOutput(
        "Superstructure/DistanceToTargetFeet", Units.metersToFeet(distanceToTargetMeters));
    Logger.recordOutput("Superstructure/IsAlignedToTarget", alignedToTarget);
    Logger.recordOutput(
        "Superstructure/ShooterReady", shooter.getCurrentState() == ShooterState.SHOOT);
    Logger.recordOutput("Superstructure/FeedReady", isFeedReadyForCurrentState());
    Logger.recordOutput(
        "Superstructure/ShootAllowed",
        SuperstructureTargeting.isShootAllowed(allianceConfirmed, currentZone));
    Logger.recordOutput(
        "Superstructure/PurgeZone",
        SuperstructureTargeting.isPurgeZone(allianceConfirmed, currentZone));

    recordShiftInfo("HubShift/Official", officialShiftInfo);
    recordShiftInfo("HubShift/Shifted", shiftedShiftInfo);
    Logger.recordOutput(
        "HubShift/FirstActiveAlliance", HubShiftUtil.getFirstActiveAlliance().name());
  }
}
