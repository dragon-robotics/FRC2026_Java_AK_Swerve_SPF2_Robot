// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import static frc.robot.subsystems.drive.DriveConstants.DEADBAND;
import static frc.robot.subsystems.drive.DriveConstants.FF_RAMP_RATE;
import static frc.robot.subsystems.drive.DriveConstants.FF_START_DELAY;
import static frc.robot.subsystems.drive.DriveConstants.HEADING_KD;
import static frc.robot.subsystems.drive.DriveConstants.HEADING_KI;
import static frc.robot.subsystems.drive.DriveConstants.HEADING_KP;
import static frc.robot.subsystems.drive.DriveConstants.ROTATION_MAX_ACCELERATION;
import static frc.robot.subsystems.drive.DriveConstants.ROTATION_MAX_VELOCITY;
import static frc.robot.subsystems.drive.DriveConstants.WHEEL_RADIUS_MAX_VELOCITY;
import static frc.robot.subsystems.drive.DriveConstants.WHEEL_RADIUS_RAMP_RATE;
import frc.robot.util.constants.FieldConstants.FieldZones;

public class DriveCommands {

  private DriveCommands() {}

  enum DriveMode {
    LOCK_ZONE,
    ROTATE,
    HOLD_HEADING
  }

  record JoystickSpeeds(double translation, double strafe, double rotation) {}

  static JoystickSpeeds processJoystickInputs(
      double rawTranslation,
      double rawStrafe,
      double rawRotation,
      boolean halfSpeed,
      double maxLinearSpeed,
      double maxAngularSpeed) {
    double translation = MathUtil.applyDeadband(rawTranslation, DEADBAND, 1.0);
    double strafe = MathUtil.applyDeadband(rawStrafe, DEADBAND, 1.0);
    double rotation = MathUtil.applyDeadband(rawRotation, DEADBAND, 1.0);

    translation = Math.copySign(translation * translation, translation);
    strafe = Math.copySign(strafe * strafe, strafe);
    rotation = Math.copySign(rotation * rotation, rotation);

    if (halfSpeed) {
      translation *= 0.45;
      strafe *= 0.45;
      rotation *= 0.45;
    }

    return new JoystickSpeeds(
        translation * maxLinearSpeed, strafe * maxLinearSpeed, rotation * maxAngularSpeed);
  }

  static DriveMode selectDriveMode(
      boolean angleLock,
      boolean zoneHeadingAvailable,
      boolean rotationTriggered,
      boolean rotationActive) {
    if (angleLock && zoneHeadingAvailable) {
      return DriveMode.LOCK_ZONE;
    }
    if (rotationTriggered || rotationActive) {
      return DriveMode.ROTATE;
    }
    return DriveMode.HOLD_HEADING;
  }

  static boolean isRotationActive(
      double rotationLastTriggered, double now, double measuredOmegaRadiansPerSecond) {
    return MathUtil.isNear(rotationLastTriggered, now, 0.1)
        && Math.abs(measuredOmegaRadiansPerSecond) > Math.toRadians(10);
  }

  static boolean shouldResetOrientationController(
      DriveMode mode, Optional<Rotation2d> storedHeading) {
    return mode != DriveMode.ROTATE && storedHeading.isEmpty();
  }

  static ChassisSpeeds toFieldRelativeSpeeds(
      ChassisSpeeds speeds, Rotation2d robotRotation, Alliance alliance) {
    boolean isFlipped = alliance == Alliance.Red;
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        speeds, isFlipped ? robotRotation.plus(new Rotation2d(Math.PI)) : robotRotation);
  }

  public static Optional<Rotation2d> getZoneLockedHeading(FieldZones zone, Alliance alliance) {
    double leftLockDegrees = alliance == Alliance.Red ? 135.0 : -45.0;
    double rightLockDegrees = alliance == Alliance.Red ? -135.0 : 45.0;

    return switch (zone) {
      case ALLIANCE_LEFT,
          NEUTRAL_LEFT_SHOOT,
          NEUTRAL_LEFT_PURGE,
          NEUTRAL_LEFT,
          OPPONENT_LEFT -> Optional.of(Rotation2d.fromDegrees(leftLockDegrees));
      case ALLIANCE_RIGHT,
          NEUTRAL_RIGHT_SHOOT,
          NEUTRAL_RIGHT_PURGE,
          NEUTRAL_RIGHT,
          OPPONENT_RIGHT -> Optional.of(Rotation2d.fromDegrees(rightLockDegrees));
      default -> Optional.empty();
    };
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(translationSup.getAsDouble(), strafeSup.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Default, field relative drive command using joystick for linear control and PID to hold current
   * orientation, but also control angular rotation if the user requests rotation Also supports
   * half-speed mode and zone-locked heading where the robot maintains a specific orientation
   * relative to areas of the field.
   */
  public static Command joystickDefaultDrive(
      Drive drive,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier omegaSupplier,
      BooleanSupplier halfSpeedSupplier,
      BooleanSupplier angleLockSup,
      Supplier<Optional<Rotation2d>> headingGetter,
      Consumer<Optional<Rotation2d>> headingSetter,
      DoubleSupplier rotationLastTriggeredGetter,
      DoubleConsumer rotationLastTriggeredSetter,
      Supplier<Optional<Rotation2d>> zoneLockedHeadingGetter) {
    ProfiledPIDController orientationController;
    orientationController =
        new ProfiledPIDController(
            HEADING_KP,
            HEADING_KI,
            HEADING_KD,
            new TrapezoidProfile.Constraints(ROTATION_MAX_VELOCITY, ROTATION_MAX_ACCELERATION));
    orientationController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              double rawRotation = omegaSupplier.getAsDouble();
              JoystickSpeeds speeds =
                  processJoystickInputs(
                      translationSup.getAsDouble(),
                      strafeSup.getAsDouble(),
                      rawRotation,
                      halfSpeedSupplier.getAsBoolean(),
                      drive.getMaxLinearSpeedMetersPerSec(),
                      drive.getMaxAngularSpeedRadPerSec());

              boolean rotationTriggered = Math.abs(rawRotation) > DEADBAND;
              if (rotationTriggered) {
                rotationLastTriggeredSetter.accept(Timer.getFPGATimestamp());
              }
              boolean rotationActive =
                  isRotationActive(
                      rotationLastTriggeredGetter.getAsDouble(),
                      Timer.getFPGATimestamp(),
                      drive.getChassisSpeeds().omegaRadiansPerSecond);

              Optional<Rotation2d> zoneHeading = zoneLockedHeadingGetter.get();
              DriveMode mode =
                  selectDriveMode(
                      angleLockSup.getAsBoolean(),
                      zoneHeading.isPresent(),
                      rotationTriggered,
                      rotationActive);

              Rotation2d currentRotation = drive.getRotation();
              Optional<Rotation2d> storedHeading = headingGetter.get();
              if (shouldResetOrientationController(mode, storedHeading)) {
                orientationController.reset(currentRotation.getRadians());
              }

              double omega = speeds.rotation();
              Optional<Rotation2d> targetHeading = Optional.empty();
              switch (mode) {
                case LOCK_ZONE:
                  headingSetter.accept(Optional.of(currentRotation));
                  targetHeading = zoneHeading;
                  break;
                case ROTATE:
                  headingSetter.accept(Optional.empty());
                  break;
                case HOLD_HEADING:
                  if (storedHeading.isEmpty()) {
                    storedHeading = Optional.of(currentRotation);
                    headingSetter.accept(storedHeading);
                  }
                  targetHeading = storedHeading;
                  break;
              }

              if (targetHeading.isPresent()) {
                omega =
                    orientationController.calculate(
                        currentRotation.getRadians(), targetHeading.get().getRadians());
              }

              ChassisSpeeds chassisSpeeds =
                  new ChassisSpeeds(speeds.translation(), speeds.strafe(), omega);
              Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
              drive.runVelocity(toFieldRelativeSpeeds(chassisSpeeds, currentRotation, alliance));
            },
            drive)
        .beforeStarting(() -> orientationController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            HEADING_KP,
            HEADING_KI,
            HEADING_KD,
            new TrapezoidProfile.Constraints(ROTATION_MAX_VELOCITY, ROTATION_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(
                      translationSup.getAsDouble(), strafeSup.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
