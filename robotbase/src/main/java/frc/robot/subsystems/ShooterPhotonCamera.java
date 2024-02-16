package frc.robot.subsystems;

import static frc.robot.Constants.SHOOTER_PHOTON_CAMERA;

import edu.wpi.first.math.geometry.Transform3d;

public class ShooterPhotonCamera extends PhotonVision {

  /**
   * An extension of the PhotonVision class that has the pipelines for the shooter photon camera
   * implimented. Uses the PhotonVision classes method to set the pipeline, so we set them
   * consistantly.
   */
  public ShooterPhotonCamera(
      String cameraName, Transform3d robotToCameraTransform, double headOnTolerance) {
    super(cameraName, robotToCameraTransform, headOnTolerance);
  }

  public void setNeuralNetworkPipelineActive() {
    super.setPipeline(SHOOTER_PHOTON_CAMERA.NEURAL_NETWORK_PIPELINE);
  }

  public void setRetroReflectivePipelineActive() {
    super.setPipeline(SHOOTER_PHOTON_CAMERA.NEURAL_NETWORK_PIPELINE);
  }

  public void setAprilTagPipelineActive() {
    super.setPipeline(SHOOTER_PHOTON_CAMERA.NEURAL_NETWORK_PIPELINE);
  }

  public void setDefaultPipelineActive() {
    super.setPipeline(SHOOTER_PHOTON_CAMERA.DEFAULT_PIPELINE);
  }
}
