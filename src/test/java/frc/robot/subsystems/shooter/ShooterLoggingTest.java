package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

class ShooterLoggingTest {
  @Test
  void periodicPublishesMeasuredFlywheelRpmAsAdvantageKitOutput()
      throws ReflectiveOperationException {
    Field runningField = Logger.class.getDeclaredField("running");
    Field entryField = Logger.class.getDeclaredField("entry");
    Field outputTableField = Logger.class.getDeclaredField("outputTable");
    runningField.setAccessible(true);
    entryField.setAccessible(true);
    outputTableField.setAccessible(true);

    boolean previousRunning = runningField.getBoolean(null);
    LogTable previousEntry = (LogTable) entryField.get(null);
    LogTable previousOutputTable = (LogTable) outputTableField.get(null);
    LogTable testEntry = new LogTable(0);
    runningField.setBoolean(null, true);
    entryField.set(null, testEntry);
    outputTableField.set(null, testEntry.getSubtable("RealOutputs"));

    try {
      Shooter shooter =
          new Shooter(
              new ShooterIO() {
                @Override
                public void updateInputs(ShooterIOInputs inputs) {
                  inputs.flywheelLeadVelocityRpm = 2468.0;
                }
              });

      shooter.periodic();

      assertEquals(
          2468.0, testEntry.get("RealOutputs/Shooter/MeasuredFlywheelRPM", Double.NaN), 1e-9);
    } finally {
      outputTableField.set(null, previousOutputTable);
      entryField.set(null, previousEntry);
      runningField.setBoolean(null, previousRunning);
    }
  }
}
