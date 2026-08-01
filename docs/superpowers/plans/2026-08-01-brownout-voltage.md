# Brownout Voltage Configuration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Configure the roboRIO brownout threshold to 5.5 volts exactly like the Spitfire v2 robot.

**Architecture:** Apply the WPILib global brownout setting once during `Robot` construction, immediately after `RobotContainer` is created. The call remains outside the runtime mode switch, so real, simulation, and replay all use the same value.

**Tech Stack:** Java 17, WPILib `RobotController`, AdvantageKit `LoggedRobot`, Gradle, Spotless, JUnit 5

## Global Constraints

- Set the brownout threshold to exactly `5.5` volts.
- Apply the setting unconditionally in real, simulation, and replay modes.
- Mirror Spitfire v2 with a direct `RobotController.setBrownoutVoltage(5.5)` call.
- Do not add a constant, helper, or mode-specific branch.
- Use the user-approved configuration-only exception to test-first development; rely on formatting, compilation, the full test suite, and diff inspection.
- Preserve the pre-existing uncommitted shooter design and plan edits.

---

### Task 1: Configure the global brownout voltage

**Files:**
- Modify: `src/main/java/frc/robot/Robot.java:10-72`

**Interfaces:**
- Consumes: `edu.wpi.first.wpilibj.RobotController.setBrownoutVoltage(double voltage)`
- Produces: a global 5.5-volt roboRIO brownout threshold configured during robot startup

- [ ] **Step 1: Confirm the target and reference locations**

Verify that this robot constructs `RobotContainer` in `Robot()` and that Spitfire v2 calls the WPILib setter after its container construction:

```powershell
rg -n "RobotContainer|setBrownoutVoltage|RobotController" src/main/java/frc/robot/Robot.java
git -C "C:/FRC_Software/FRC 2026 Software/Projects/FRC2026_Java_Swerve_Robot" show spitfire-v2:src/main/java/frc/robot/Robot.java
```

Expected: this robot has no brownout setter; Spitfire v2 contains `RobotController.setBrownoutVoltage(5.5);` after `new RobotContainer()`.

- [ ] **Step 2: Implement the minimal startup configuration**

Add the WPILib import:

```java
import edu.wpi.first.wpilibj.RobotController;
```

Then add the direct setter immediately after `robotContainer = new RobotContainer();`:

```java
robotContainer = new RobotContainer();

// Set brownout voltage to 5.5V
RobotController.setBrownoutVoltage(5.5);
```

- [ ] **Step 3: Apply formatting and run the full verification gate**

Run:

```powershell
.\gradlew.bat spotlessApply check --console=plain --no-daemon
```

Expected: `BUILD SUCCESSFUL` and every JUnit test passes.

- [ ] **Step 4: Inspect the final diff and repository state**

Run:

```powershell
git diff --check
git diff -- src/main/java/frc/robot/Robot.java
git status --short
```

Expected: `Robot.java` contains only the new import and constructor call; no whitespace errors appear; the two pre-existing shooter documentation edits remain present and unstaged.

- [ ] **Step 5: Commit the implementation**

Stage only `Robot.java` and commit:

```powershell
git add -- src/main/java/frc/robot/Robot.java
git commit -m "config: lower brownout voltage to 5.5 volts"
```
