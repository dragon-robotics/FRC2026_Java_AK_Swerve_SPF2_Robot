// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Superstate;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionDriveBindings;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVision.HeadingProvider;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionRuntimeConfig;
import frc.robot.subsystems.vision.VisionSimulation;
import frc.robot.util.constants.OperatorConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.DoubleFunction;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  record DriveVisionConstruction<D, V>(D drive, V vision) {}

  private record VisionStack(Vision vision, VisionSimulation simulation) {}

  record VisionIOFactory(
      String cameraName,
      VisionSimulation simulationOwner,
      Class<? extends VisionIO> implementationType,
      Supplier<? extends VisionIO> constructor) {
    VisionIO create() {
      return constructor.get();
    }
  }

  record ShooterIOFactory<T extends ShooterIO>(
      Class<T> implementationType, Supplier<T> constructor) {
    T create() {
      return constructor.get();
    }
  }

  record IntakeIOFactory<T extends IntakeIO>(Class<T> implementationType, Supplier<T> constructor) {
    T create() {
      return constructor.get();
    }
  }

  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Hopper hopper;
  private final Shooter shooter;
  private final Superstructure superstructure;
  private final Vision vision;
  private final VisionSimulation visionSimulation;

  // Controller

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this(drive -> VisionRuntimeConfig.fromSystemProperties());
  }

  RobotContainer(Function<Drive, VisionRuntimeConfig> visionConfigConstructor) {
    Constants.Mode mode = Constants.currentMode;
    DriveVisionConstruction<Drive, VisionStack> driveVision =
        constructDriveThenVision(
            () -> createDrive(mode),
            visionConfigConstructor,
            (constructedDrive, config) -> createVisionStack(mode, constructedDrive, config));
    drive = driveVision.drive();
    vision = driveVision.vision().vision();
    visionSimulation = driveVision.vision().simulation();

    intake = new Intake(createIntakeIO(Constants.currentMode));
    hopper = new Hopper(createHopperIO(Constants.currentMode));
    shooter = new Shooter(createShooterIO(Constants.currentMode));
    superstructure = new Superstructure(drive, intake, hopper, shooter, vision);

    // Set up auto routines
    autoChooser =
        registerThenBuildChooser(
            () -> registerNamedCommands(superstructure),
            () -> new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser()));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, field-relative drive with heading hold, free rotation, half speed, and zone
    // lock
    drive.setDefaultCommand(
        DriveCommands.joystickDefaultDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> driverController.getHID().getPOV() == 0,
            () -> driverController.getHID().getXButton(),
            superstructure::getCurrentHeading,
            superstructure::setCurrentHeading,
            superstructure::getRotationLastTriggered,
            superstructure::setRotationLastTriggered,
            superstructure::getZoneLockedHeading));

    // Reset gyro to 0° when start and back buttons are pressed
    driverController
        .start()
        .and(driverController.back())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true)
                .withName("Driver Heading Reset"));

    operatorController
        .start()
        .and(operatorController.back())
        .onTrue(
            Commands.runOnce(vision::forceReseedFromVision, vision)
                .withName("Force Vision Reseed"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  static <D, C, V> DriveVisionConstruction<D, V> constructDriveThenVision(
      Supplier<D> driveConstructor,
      Function<D, C> configConstructor,
      BiFunction<D, C, V> visionConstructor) {
    D constructedDrive = Objects.requireNonNull(driveConstructor.get());
    C config = Objects.requireNonNull(configConstructor.apply(constructedDrive));
    V constructedVision = Objects.requireNonNull(visionConstructor.apply(constructedDrive, config));
    return new DriveVisionConstruction<>(constructedDrive, constructedVision);
  }

  static void registerNamedCommands(Superstructure superstructure) {
    NamedCommands.registerCommand("Intake", superstructure.setStateCmd(Superstate.INTAKE));
    NamedCommands.registerCommand("Shoot", superstructure.shootWithJuicerDelayCmd());
    NamedCommands.registerCommand("ShootNoAim", superstructure.shootNoAimWithJuicerDelayCmd());
    NamedCommands.registerCommand("Drive", superstructure.setStateCmd(Superstate.DRIVE));
  }

  static <T> T registerThenBuildChooser(Runnable register, Supplier<T> chooserFactory) {
    Objects.requireNonNull(register).run();
    return Objects.requireNonNull(chooserFactory).get();
  }

  private static Drive createDrive(Constants.Mode mode) {
    return switch (mode) {
      case REAL ->
      // ModuleIOTalonFX uses TalonFX drive/turn motors and a CANcoder.
      new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight));
      case SIM -> new Drive(
          new GyroIO() {},
          new ModuleIOSim(TunerConstants.FrontLeft),
          new ModuleIOSim(TunerConstants.FrontRight),
          new ModuleIOSim(TunerConstants.BackLeft),
          new ModuleIOSim(TunerConstants.BackRight));
      case REPLAY -> new Drive(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
    };
  }

  private static VisionStack createVisionStack(
      Constants.Mode mode, Drive drive, VisionRuntimeConfig config) {
    VisionConstructionPlan plan = VisionConstructionPlan.forMode(mode);
    VisionDriveBindings driveBindings = VisionDriveBindings.fromDrive(drive);
    HeadingProvider headingProvider =
        visionHeadingProvider(drive::samplePoseAt, drive::getChassisSpeeds);
    VisionSimulation simulation =
        mode == Constants.Mode.SIM ? new VisionSimulation(drive::getSimulationPose) : null;
    List<VisionIOFactory> factories = visionIOFactories(plan, config, headingProvider, simulation);
    VisionIO[] io = factories.stream().map(VisionIOFactory::create).toArray(VisionIO[]::new);
    Runnable simulationUpdate = simulation == null ? null : simulation::update;
    Vision vision =
        new Vision(driveBindings, config, visionPreInputHook(mode, simulationUpdate), io);
    initializeVisionAiming(vision::setAiming);
    return new VisionStack(vision, simulation);
  }

  static List<VisionIOFactory> visionIOFactories(
      VisionConstructionPlan plan,
      VisionRuntimeConfig config,
      HeadingProvider headingProvider,
      VisionSimulation simulation) {
    if (plan.ioKind() == VisionConstructionPlan.IoKind.SIM_PHOTON) {
      Objects.requireNonNull(simulation, "SIM vision requires one shared simulation owner");
    }

    var factories = new ArrayList<VisionIOFactory>(plan.cameras().size());
    for (var camera : plan.cameras()) {
      VisionIOFactory factory =
          switch (plan.ioKind()) {
            case REAL_PHOTON -> visionIOFactory(
                camera.name(),
                VisionIOPhotonVision.class,
                null,
                () -> new VisionIOPhotonVision(camera, config, headingProvider));
            case SIM_PHOTON -> visionIOFactory(
                camera.name(),
                VisionIOPhotonVisionSim.class,
                simulation,
                () -> new VisionIOPhotonVisionSim(camera, config, headingProvider, simulation));
            case REPLAY_NOOP -> visionIOFactory(
                camera.name(), VisionIO.NoOp.class, null, () -> new VisionIO.NoOp(camera.name()));
          };
      factories.add(factory);
    }
    return List.copyOf(factories);
  }

  private static VisionIOFactory visionIOFactory(
      String cameraName,
      Class<? extends VisionIO> implementationType,
      VisionSimulation simulationOwner,
      Supplier<? extends VisionIO> constructor) {
    return new VisionIOFactory(cameraName, simulationOwner, implementationType, constructor);
  }

  static HeadingProvider visionHeadingProvider(
      DoubleFunction<Optional<Pose2d>> timestampedPose, Supplier<ChassisSpeeds> measuredSpeeds) {
    return new HeadingProvider() {
      @Override
      public Optional<Rotation2d> headingAt(double fpgaTimestampSeconds) {
        return timestampedPose.apply(fpgaTimestampSeconds).map(Pose2d::getRotation);
      }

      @Override
      public Optional<Pose3d> seedPoseAt(double fpgaTimestampSeconds) {
        return timestampedPose.apply(fpgaTimestampSeconds).map(Pose3d::new);
      }

      @Override
      public double angularRateRadPerSecond() {
        return measuredSpeeds.get().omegaRadiansPerSecond;
      }

      @Override
      public double linearSpeedMetersPerSecond() {
        ChassisSpeeds speeds = measuredSpeeds.get();
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      }
    };
  }

  static Runnable visionPreInputHook(Constants.Mode mode, Runnable simulationUpdate) {
    return mode == Constants.Mode.SIM ? Objects.requireNonNull(simulationUpdate) : () -> {};
  }

  static void initializeVisionAiming(Consumer<Boolean> setAiming) {
    setAiming.accept(false);
  }

  static void setVisionSimulationPoseSupplier(
      VisionSimulation simulation, Supplier<Pose2d> poseSupplier) {
    if (simulation != null) {
      simulation.setPoseSupplier(poseSupplier);
    }
  }

  /** Overrides the shared Photon simulation truth pose; this is a no-op outside SIM. */
  public void setVisionSimulationPoseSupplier(Supplier<Pose2d> poseSupplier) {
    setVisionSimulationPoseSupplier(visionSimulation, poseSupplier);
  }

  Vision vision() {
    return vision;
  }

  Drive drive() {
    return drive;
  }

  VisionSimulation visionSimulation() {
    return visionSimulation;
  }

  int driverControllerPort() {
    return driverController.getHID().getPort();
  }

  int operatorControllerPort() {
    return operatorController.getHID().getPort();
  }

  static HopperIO createHopperIO(Constants.Mode mode) {
    return switch (mode) {
      case REAL -> new HopperIOTalonFX();
      case SIM -> new HopperIOSim();
      case REPLAY -> new HopperIO.NoOp();
    };
  }

  static ShooterIO createShooterIO(Constants.Mode mode) {
    return shooterIOFactory(mode).create();
  }

  static ShooterIOFactory<? extends ShooterIO> shooterIOFactory(Constants.Mode mode) {
    return switch (mode) {
      case REAL -> new ShooterIOFactory<>(ShooterIOTalonFX.class, ShooterIOTalonFX::new);
      case SIM -> new ShooterIOFactory<>(ShooterIOSim.class, ShooterIOSim::new);
      case REPLAY -> new ShooterIOFactory<>(ShooterIO.NoOp.class, ShooterIO.NoOp::new);
    };
  }

  static IntakeIO createIntakeIO(Constants.Mode mode) {
    return intakeIOFactory(mode).create();
  }

  static IntakeIOFactory<? extends IntakeIO> intakeIOFactory(Constants.Mode mode) {
    return switch (mode) {
      case REAL -> new IntakeIOFactory<>(IntakeIOTalonFX.class, IntakeIOTalonFX::new);
      case SIM -> new IntakeIOFactory<>(IntakeIOSim.class, IntakeIOSim::new);
      case REPLAY -> new IntakeIOFactory<>(IntakeIO.NoOp.class, IntakeIO.NoOp::new);
    };
  }
}
