package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Constants.SHOOTER_PHOTON_CAMERA;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ShooterPhotonCamera extends PhotonVisionCamera {

  /**
   * An extension of the PhotonVision class that has the pipelines for the shooter photon camera
   * implimented. Uses the PhotonVision classes method to set the pipeline, so we set them
   * consistently.
   */
  public ShooterPhotonCamera(
      String cameraName, Transform3d robotToCameraTransform, double headOnTolerance) {
    super(cameraName, robotToCameraTransform, headOnTolerance);
  }

  public void setNeuralNetworkPipelineActive() {
    super.setPipeline(SHOOTER_PHOTON_CAMERA.NEURAL_NETWORK_PIPELINE);
  }

  public void setAprilTagPipelineActive() {
    super.setPipeline(SHOOTER_PHOTON_CAMERA.APRIL_TAG_PIPELINE);
  }

  public void setDefaultPipelineActive() {
    super.setPipeline(SHOOTER_PHOTON_CAMERA.DEFAULT_PIPELINE);
  }

  public PhotonTrackedTarget getSpeakerTarget() {
    var targets = super.filterAprilTags(Constants.APRIL_TAG_IDS.SPEAKER_CENTER_TAGS);
    if (targets == null || targets.isEmpty()) {
      return null;
    }
    return targets.get(0);
  }

  public PhotonTrackedTarget getAmpTarget() {
    var targets = super.filterAprilTags(Constants.APRIL_TAG_IDS.SPEAKER_CENTER_TAGS);
    if (targets == null || !targets.isEmpty()) {
      return null;
    }
    return targets.get(0);
  }
}
