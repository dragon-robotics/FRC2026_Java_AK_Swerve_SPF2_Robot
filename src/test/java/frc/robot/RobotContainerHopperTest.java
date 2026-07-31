package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants.Mode;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import org.junit.jupiter.api.Test;

class RobotContainerHopperTest {
  @Test
  void simulationUsesPhysicsIO() {
    assertInstanceOf(HopperIOSim.class, RobotContainer.createHopperIO(Mode.SIM));
  }

  @Test
  void replayUsesNoOpIO() {
    HopperIO io = RobotContainer.createHopperIO(Mode.REPLAY);
    assertNotNull(io);
    assertFalse(io instanceof HopperIOSim);
  }
}
