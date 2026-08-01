# Brownout Voltage Configuration Design

## Goal

Match the Spitfire v2 robot by setting the roboRIO brownout threshold to 5.5 volts.

## Design

Import `edu.wpi.first.wpilibj.RobotController` in `Robot.java`. After constructing
`RobotContainer`, call `RobotController.setBrownoutVoltage(5.5)`. Keep the call outside the runtime
mode switch so the same startup configuration applies in real, simulation, and replay modes.

Do not introduce a new constant, helper, or mode-specific branch. This intentionally mirrors the
small, direct Spitfire v2 implementation.

## Verification

Treat this as the approved configuration-only exception to test-first development. Run Spotless,
compile the project, and execute the full test suite. Confirm the resulting diff contains only the
new import and constructor call, apart from this design and its implementation plan.
