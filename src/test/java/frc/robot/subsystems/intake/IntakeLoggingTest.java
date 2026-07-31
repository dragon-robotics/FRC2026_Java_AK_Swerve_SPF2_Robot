package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

class IntakeLoggingTest {
  @Test
  void periodicPublishesMeasuredArmAndStateOutputs() throws ReflectiveOperationException {
    Field runningField = Logger.class.getDeclaredField("running");
    Field entryField = Logger.class.getDeclaredField("entry");
    Field outputTableField = Logger.class.getDeclaredField("outputTable");
    runningField.setAccessible(true);
    entryField.setAccessible(true);
    outputTableField.setAccessible(true);
    boolean previousRunning = runningField.getBoolean(null);
    LogTable previousEntry = (LogTable) entryField.get(null);
    LogTable previousOutput = (LogTable) outputTableField.get(null);
    LogTable testEntry = new LogTable(0);
    runningField.setBoolean(null, true);
    entryField.set(null, testEntry);
    outputTableField.set(null, testEntry.getSubtable("RealOutputs"));
    try {
      Intake intake =
          new Intake(
              new IntakeIO() {
                @Override
                public void updateInputs(IntakeIOInputs inputs) {
                  inputs.armPositionRotations = 0.37;
                }
              },
              () -> false);
      intake.periodic();
      assertEquals(
          0.37, testEntry.get("RealOutputs/Intake/MeasuredArmRotations", Double.NaN), 1e-9);
      assertEquals("HOME", testEntry.get("RealOutputs/Intake/CurrentState", ""));
    } finally {
      outputTableField.set(null, previousOutput);
      entryField.set(null, previousEntry);
      runningField.setBoolean(null, previousRunning);
    }
  }
}
