package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Constants.SHOOTER_PHOTON_CAMERA;

public class ShooterPhotonCamera extends PhotonVisionCamera {

  /**
   * An extension of the PhotonVision class that has the pipelines for the shooter photon camera
   * implimented. Uses the PhotonVision classes method to set the pipeline, so we set them
   * consistently.
   */
  public ShooterPhotonCamera(
      String cameraName, Transform3d robotToCameraTransform, double headOnTolerance) {
    super(cameraName, robotToCameraTransform, headOnTolerance, SHOOTER_PHOTON_CAMERA.LOOPS_TO_KEEP_CACHED_DATA_VALID);
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

  /**
   * @return The yaw of any seen speaker april tag. WIll return NaN if it doesnt see one or doesnt have one cached.
   */
  public double getSpeakerTargetYaw(){
    for(int targetID : Constants.APRIL_TAG_IDS.SPEAKER_CENTER_TAGS){
      var yaw = super.getTargetYaw(targetID);
      if(yaw != Double.NaN){
        return yaw;
      }
    }
    return Double.NaN;
  }

  /**
   * @return The pitch of any seen speaker april tag. WIll return NaN if it doesnt see one or doesnt have one cached.
   */
  public double getSpeakerTargetPitch(){
    for(int targetID : Constants.APRIL_TAG_IDS.SPEAKER_CENTER_TAGS){
      var yaw = super.getTargetPitch(targetID);
      if(yaw != Double.NaN){
        return yaw;
      }
    }
    return Double.NaN;
  }

  /**
   * @return The yaw of any seen amp april tag. WIll return NaN if it doesnt see one or doesnt have one cached.
   */
  public double getAmpTargetYaw(){
    for(int targetID : Constants.APRIL_TAG_IDS.AMP_TAGS){
      var yaw = super.getTargetYaw(targetID);
      if(yaw != Double.NaN){
        return yaw;
      }
    }
    return Double.NaN;
  }

  /**
   * @return The pitch of any seen speaker april tag. WIll return NaN if it doesnt see one or doesnt have one cached.
   */
  public double getAmpTargetPitch(){
    for(int targetID : Constants.APRIL_TAG_IDS.AMP_TAGS){
      var yaw = super.getTargetPitch(targetID);
      if(yaw != Double.NaN){
        return yaw;
      }
    }
    return Double.NaN;
  }
}
