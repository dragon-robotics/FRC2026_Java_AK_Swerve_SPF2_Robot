package frc.robot;

import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.List;

/** Pure mapping from robot runtime mode to the deferred four-camera vision construction plan. */
record VisionConstructionPlan(IoKind ioKind, List<CameraConfig> cameras) {
  enum IoKind {
    REAL_PHOTON,
    SIM_PHOTON,
    REPLAY_NOOP
  }

  VisionConstructionPlan {
    cameras = List.copyOf(cameras);
  }

  static VisionConstructionPlan forMode(Constants.Mode mode) {
    IoKind ioKind =
        switch (mode) {
          case REAL -> IoKind.REAL_PHOTON;
          case SIM -> IoKind.SIM_PHOTON;
          case REPLAY -> IoKind.REPLAY_NOOP;
        };
    return new VisionConstructionPlan(ioKind, VisionConstants.CAMERAS);
  }
}
