package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Receives one timestamped vision pose and its estimator standard deviations. */
@FunctionalInterface
public interface VisionMeasurementConsumer {
  void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations);
}
