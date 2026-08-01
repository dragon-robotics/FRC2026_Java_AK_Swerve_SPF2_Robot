package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Drive callbacks and providers consumed by the vision subsystem. */
public record VisionDriveBindings(
    VisionMeasurementConsumer measurementConsumer,
    Consumer<Pose2d> estimatorReset,
    Supplier<Pose2d> currentPose,
    DoubleFunction<Optional<Pose2d>> timestampedPose,
    Supplier<ChassisSpeeds> chassisSpeeds,
    DoubleSupplier pitchDegrees,
    DoubleSupplier rollDegrees) {
  public static VisionDriveBindings fromDrive(Drive drive) {
    return new VisionDriveBindings(
        drive::addVisionMeasurement,
        drive::setPose,
        drive::getPose,
        drive::samplePoseAt,
        drive::getChassisSpeeds,
        drive::getPitchDegrees,
        drive::getRollDegrees);
  }
}
