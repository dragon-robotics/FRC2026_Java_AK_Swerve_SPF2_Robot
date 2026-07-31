package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants.Mode;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import org.junit.jupiter.api.Test;

class RobotContainerIntakeTest {
  @Test
  void realFactoryMappingNamesHardwareIoWithoutConstructingIt() {
    assertEquals(
        IntakeIOTalonFX.class, RobotContainer.intakeIOFactory(Mode.REAL).implementationType());
  }

  @Test
  void simulationUsesFullPhysicsIo() {
    assertInstanceOf(IntakeIOSim.class, RobotContainer.createIntakeIO(Mode.SIM));
  }

  @Test
  void replayUsesNoOpIo() {
    assertInstanceOf(IntakeIO.NoOp.class, RobotContainer.createIntakeIO(Mode.REPLAY));
  }
}
