package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants.Mode;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import org.junit.jupiter.api.Test;

class RobotContainerShooterTest {
  @Test
  void simulationUsesFullPhysicsIo() {
    assertInstanceOf(ShooterIOSim.class, RobotContainer.createShooterIO(Mode.SIM));
  }

  @Test
  void replayUsesNoOpIo() {
    assertInstanceOf(ShooterIO.NoOp.class, RobotContainer.createShooterIO(Mode.REPLAY));
  }
}
