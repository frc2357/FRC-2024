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
    super(cameraName, robotToCameraTransform);
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
   * @return The yaw of any seen speaker april tag. Will return NaN if it doesnt see one or doesnt
   *     have one cached.
   */
  public double getSpeakerTargetYaw() {
    return super.getTargetYaw(
        Constants.APRIL_TAG_IDS.SPEAKER_CENTER_TAGS,
        SHOOTER_PHOTON_CAMERA.SPEAKER_TARGET_TIMEOUT_MS);
  }

  /**
   * @return The pitch of any seen speaker april tag. Will return NaN if it doesnt see one or doesnt
   *     have one cached.
   */
  public double getSpeakerTargetPitch() {
    return super.getTargetPitch(
        Constants.APRIL_TAG_IDS.SPEAKER_CENTER_TAGS,
        SHOOTER_PHOTON_CAMERA.SPEAKER_TARGET_TIMEOUT_MS);
  }

  /**
   * @return The yaw of any seen amp april tag. Will return NaN if it doesnt see one or doesnt have
   *     one cached.
   */
  public double getAmpTargetYaw() {
    return super.getTargetYaw(
        Constants.APRIL_TAG_IDS.AMP_TAGS, SHOOTER_PHOTON_CAMERA.AMP_TARGET_TIMEOUT_MS);
  }

  /**
   * @return The pitch of any seen amp april tag. Will return NaN if it doesnt see one or doesnt
   *     have one cached.
   */
  public double getAmpTargetPitch() {
    return super.getTargetPitch(
        Constants.APRIL_TAG_IDS.AMP_TAGS, SHOOTER_PHOTON_CAMERA.AMP_TARGET_TIMEOUT_MS);
  }

  /**
   * @return The yaw of any seen amp april tag. Will return NaN if it doesnt see one or doesnt have
   *     one cached.
   */
  public double getStageTargetYaw() {
    return super.getTargetYaw(
        Constants.APRIL_TAG_IDS.STAGE_TAGS, SHOOTER_PHOTON_CAMERA.STAGE_TARGET_TIMEOUT_MS);
  }

  /**
   * @return The pitch of any seen amp april tag. Will return NaN if it doesnt see one or doesnt
   *     have one cached.
   */
  public double getStageTargetPitch() {
    return super.getTargetPitch(
        Constants.APRIL_TAG_IDS.STAGE_TAGS, SHOOTER_PHOTON_CAMERA.STAGE_TARGET_TIMEOUT_MS);
  }

  /**
   * @return The yaw of any seen amp april tag. Will return NaN if it doesnt see one or doesnt have
   *     one cached.
   */
  public double getRightStageTargetYaw() {
    return super.getTargetYaw(
        Constants.APRIL_TAG_IDS.RIGHT_STAGE_TAGS, SHOOTER_PHOTON_CAMERA.STAGE_TARGET_TIMEOUT_MS);
  }

  /**
   * @return The pitch of any seen amp april tag. Will return NaN if it doesnt see one or doesnt
   *     have one cached.
   */
  public double getRightStageTargetPitch() {
    return super.getTargetPitch(
        Constants.APRIL_TAG_IDS.RIGHT_STAGE_TAGS, SHOOTER_PHOTON_CAMERA.STAGE_TARGET_TIMEOUT_MS);
  }

  /**
   * @return The yaw of any seen amp april tag. Will return NaN if it doesnt see one or doesnt have
   *     one cached.
   */
  public double getLeftStageTargetYaw() {
    return super.getTargetYaw(
        Constants.APRIL_TAG_IDS.LEFT_STAGE_TAGS, SHOOTER_PHOTON_CAMERA.STAGE_TARGET_TIMEOUT_MS);
  }

  /**
   * @return The pitch of any seen amp april tag. Will return NaN if it doesnt see one or doesnt
   *     have one cached.
   */
  public double getLeftStageTargetPitch() {
    return super.getTargetPitch(
        Constants.APRIL_TAG_IDS.LEFT_STAGE_TAGS, SHOOTER_PHOTON_CAMERA.STAGE_TARGET_TIMEOUT_MS);
  }

  /**
   * @return The yaw of any seen amp april tag. Will return NaN if it doesnt see one or doesnt have
   *     one cached.
   */
  public double getCenterStageTargetYaw() {
    return super.getTargetYaw(
        Constants.APRIL_TAG_IDS.CENTER_STAGE_TAGS, SHOOTER_PHOTON_CAMERA.STAGE_TARGET_TIMEOUT_MS);
  }

  /**
   * @return The pitch of any seen amp april tag. Will return NaN if it doesnt see one or doesnt
   *     have one cached.
   */
  public double getCenterStageTargetPitch() {
    return super.getTargetPitch(
        Constants.APRIL_TAG_IDS.CENTER_STAGE_TAGS, SHOOTER_PHOTON_CAMERA.STAGE_TARGET_TIMEOUT_MS);
  }
}
