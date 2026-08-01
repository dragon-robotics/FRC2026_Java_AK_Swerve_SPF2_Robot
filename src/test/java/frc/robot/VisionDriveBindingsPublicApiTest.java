package frc.robot;

import static org.junit.jupiter.api.Assertions.assertSame;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.vision.VisionDriveBindings;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class VisionDriveBindingsPublicApiTest {
  @Test
  void publicConstructorAndMeasurementAccessorAreUsableOutsideVisionPackage() {
    AtomicReference<Pose2d> acceptedPose = new AtomicReference<>();
    VisionDriveBindings bindings =
        new VisionDriveBindings(
            (pose, timestampSeconds, standardDeviations) -> acceptedPose.set(pose),
            pose -> {},
            Pose2d::new,
            timestampSeconds -> Optional.of(new Pose2d()),
            ChassisSpeeds::new,
            () -> 0.0,
            () -> 0.0);
    Pose2d expected = new Pose2d(1.0, 2.0, new Rotation2d(0.3));

    bindings.measurementConsumer().accept(expected, 1.25, VecBuilder.fill(1.0, 1.0, 1.0));

    assertSame(expected, acceptedPose.get());
  }
}
