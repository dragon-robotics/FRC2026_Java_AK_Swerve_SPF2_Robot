package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import org.photonvision.PhotonCamera;

/** Real Photon decoder backed by a camera registered with one shared simulation owner. */
public final class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  public VisionIOPhotonVisionSim(
      CameraConfig cameraConfig,
      VisionRuntimeConfig runtimeConfig,
      HeadingProvider headingProvider,
      VisionSimulation simulation) {
    this(
        cameraConfig,
        runtimeConfig,
        headingProvider,
        simulation,
        new PhotonCamera(cameraConfig.name()));
  }

  private VisionIOPhotonVisionSim(
      CameraConfig cameraConfig,
      VisionRuntimeConfig runtimeConfig,
      HeadingProvider headingProvider,
      VisionSimulation simulation,
      PhotonCamera camera) {
    super(cameraConfig, runtimeConfig, camera, headingProvider);
    try {
      simulation.registerCamera(camera, cameraConfig);
    } catch (RuntimeException | Error exception) {
      camera.close();
      throw exception;
    }
  }
}
