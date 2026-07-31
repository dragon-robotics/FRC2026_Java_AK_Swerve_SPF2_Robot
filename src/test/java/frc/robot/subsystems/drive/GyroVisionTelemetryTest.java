package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.junit.jupiter.api.Test;

class GyroVisionTelemetryTest {
  @Test
  void freshInputsDefaultPitchAndRollToZero() {
    GyroIO.GyroIOInputs inputs = new GyroIO.GyroIOInputs();

    assertEquals(0.0, inputs.pitchDegrees, 1e-9);
    assertEquals(0.0, inputs.rollDegrees, 1e-9);
  }

  @Test
  void noOpSimulationGyroLeavesFreshPitchAndRollExplicitlyZero() {
    GyroIO gyroIO = new GyroIO() {};
    GyroIO.GyroIOInputs inputs = new GyroIO.GyroIOInputs();

    gyroIO.updateInputs(inputs);

    assertEquals(0.0, inputs.pitchDegrees, 1e-9);
    assertEquals(0.0, inputs.rollDegrees, 1e-9);
  }

  @Test
  void pigeonRefreshGroupAndPitchRollFrequencyPreserveVisionTelemetryContract() {
    StatusSignal<Angle> yaw = signal(Angle.class, Rotations::of);
    StatusSignal<AngularVelocity> yawVelocity = signal(AngularVelocity.class, RPM::of);
    StatusSignal<Angle> pitch = signal(Angle.class, Rotations::of);
    StatusSignal<Angle> roll = signal(Angle.class, Rotations::of);

    assertArrayEquals(
        new BaseStatusSignal[] {yaw, yawVelocity, pitch, roll},
        GyroIOPigeon2.createRefreshSignals(yaw, yawVelocity, pitch, roll));
    assertEquals(50.0, GyroIOPigeon2.createStatusFrequencyConfig().pitchRollHz(), 1e-9);
  }

  private static <T> StatusSignal<T> signal(
      Class<T> valueClass, java.util.function.DoubleFunction<T> converter) {
    return new StatusSignal<>(StatusCode.StatusCodeNotInitialized, valueClass, converter);
  }
}
