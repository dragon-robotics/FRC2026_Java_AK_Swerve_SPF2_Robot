package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SuperstructureTestHarness;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class RobotContainerSuperstructureTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void namedCommandsRegisterTheFourRequiredSuperstructureCommands() {
    NamedCommands.clearAll();
    try (var harness = new SuperstructureTestHarness()) {
      RobotContainer.registerNamedCommands(harness.superstructure);

      assertTrue(NamedCommands.hasCommand("Intake"));
      assertTrue(NamedCommands.hasCommand("Shoot"));
      assertTrue(NamedCommands.hasCommand("ShootNoAim"));
      assertTrue(NamedCommands.hasCommand("Drive"));
    } finally {
      NamedCommands.clearAll();
    }
  }

  @Test
  void namedCommandRegistrationPrecedesChooserConstruction() {
    List<String> events = new ArrayList<>();

    RobotContainer.registerThenBuildChooser(
        () -> events.add("register"),
        () -> {
          events.add("chooser");
          return Commands.none();
        });

    assertEquals(List.of("register", "chooser"), events);
  }
}
