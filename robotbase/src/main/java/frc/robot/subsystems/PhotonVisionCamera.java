package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PHOTON_VISION;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Controls the photon vision camera options. */
public class PhotonVisionCamera extends SubsystemBase {

  private static class TargetInfo {
    public double yaw = Double.NaN;
    public double pitch = Double.NaN;
    public long timestamp = 0;
  }

  // all of these are protected so we can use them in the extended classes
  // which are only extended so we can control which pipelines we are using.
  protected PhotonCamera m_camera;
  protected PhotonPipelineResult m_result;
  protected final TargetInfo[] m_targetInfo;
  protected PhotonPoseEstimator m_poseEstimator;
  protected final Transform3d ROBOT_TO_CAMERA_TRANSFORM; // if this changes, we have bigger issues.
  protected boolean m_connectionLost;
  protected int m_bestTargetFiducialId;

  /**
   * Sets the camera stream.
   *
   * @param cameraName Name of the cameras Photon Vision network table. MUST match the net tables
   *     name, or it wont work.
   * @param robotToCameraTransform The Transform3d of the robots coordinate center to the camera.
   * @param headOnTolerance The tolerance for declaring whether or not the camera is facing a target
   *     head on.
   */
  public PhotonVisionCamera(String cameraName, Transform3d robotToCameraTransform) {
    m_camera = new PhotonCamera(cameraName);
    ROBOT_TO_CAMERA_TRANSFORM = robotToCameraTransform;

    // 0 is for note detection, 1-16 correspond to apriltag fiducial IDs
    m_targetInfo = new TargetInfo[17];
    for (int i = 0; i < m_targetInfo.length; i++) {
      m_targetInfo[i] = new TargetInfo();
    }
  }

  public void configure() {
    setDriverModeActive();
    m_poseEstimator =
        new PhotonPoseEstimator(
            PHOTON_VISION.APRIL_TAG_FIELD_LAYOUT,
            PHOTON_VISION.POSE_STRATEGY,
            m_camera,
            ROBOT_TO_CAMERA_TRANSFORM);
  }

  /**
   * Fetches the latest pipeline result.
   *
   * <p>YOU SHOULD NEVER CALL THIS! This is for the Robot periodic ONLY. NEVER call this method
   * outside of it.
   */
  public void updateResult() {
    if (!m_camera.isConnected()  && !m_connectionLost) {
      m_connectionLost = true;
      DriverStation.reportError(PHOTON_VISION.LOST_CONNECTION_ERROR_MESSAGE, false);
      return;
    }
    if (!m_result.hasTargets()) {
      return;
    }
    m_result = m_camera.getLatestResult();
    m_bestTargetFiducialId = m_result.getBestTarget().getFiducialId();
    if (m_bestTargetFiducialId == -1) {
      // this means that were doing object detection, so a different method is used.
      cacheForGamepeices(m_result.targets);
    } else {
      cacheForAprilTags(m_result.targets);
    }
    if (m_connectionLost) {
      m_connectionLost = false;
      DriverStation.reportWarning(PHOTON_VISION.CONNECTION_REGAINED_NOFICATION_MESSAGE, false);
    }
  }

  private void cacheForGamepeices(List<PhotonTrackedTarget> targetList) {
    long now = System.currentTimeMillis();
    PhotonTrackedTarget bestTarget = calculateBestGamepeiceTarget(targetList);
    TargetInfo targetInfo = m_targetInfo[0];
    targetInfo.yaw = bestTarget.getYaw();
    targetInfo.pitch = bestTarget.getPitch();
    targetInfo.timestamp = now;
  }

  private void cacheForAprilTags(List<PhotonTrackedTarget> targetList) {
    long now = System.currentTimeMillis();
    for (PhotonTrackedTarget targetSeen : targetList) {
      int id = targetSeen.getFiducialId();
      TargetInfo targetInfo = m_targetInfo[id];
      // System.out.println(targetSeen.getFiducialId());
      targetInfo.yaw = targetSeen.getYaw();
      targetInfo.pitch = targetSeen.getPitch();
      targetInfo.timestamp = now;
    }
  }

  public PhotonTrackedTarget calculateBestGamepeiceTarget(List<PhotonTrackedTarget> targetList){
    double highestPitch =
        targetList.get(0).getPitch() + PHOTON_VISION.BEST_TARGET_PITCH_TOLERANCE_DEGREES;
    PhotonTrackedTarget bestTarget = targetList.get(0);
    for (PhotonTrackedTarget targetSeen : targetList) {
      if (targetSeen.getPitch() < highestPitch
          && Math.abs(targetSeen.getYaw()) < Math.abs(bestTarget.getYaw())) {
        bestTarget = targetSeen;
      }
      // System.out.println(targetSeen.getFiducialId());
    }
    return bestTarget;
  }

  /**
   * @return Whether or not the camera is connected.
   */
  public boolean isConnected() {
    return m_connectionLost; // uses this because it will be checked every loop
  }

  /**
   * @return The current pipelines latency in milliseconds. Returns NaN if the camera is not
   *     connected.
   */
  public double getLatencyMillis() {
    return isConnected() ? m_result.getLatencyMillis() : Double.NaN;
  }

  /**
   * @return The timestamp of the latest pipeline result in seconds. Returns Double.NaN if the
   *     camera is not connected.
   */
  public double getTimestampSeconds() {
    return isConnected() ? m_result.getTimestampSeconds() : Double.NaN;
  }

  /**
   * Compares the current system time to the last cached timestamp and sees if it is older than is
   * acceptable.
   *
   * @param fiducialId Fiducial ID of the desired target to valid the data of. Notes have a
   *     fiducialId of 0
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return If the camera has seen the target within the timeout given
   */
  public boolean isValidTarget(int fiducialId, long timeoutMs) {
    long now = System.currentTimeMillis();
    long then = now - timeoutMs;

    TargetInfo target = m_targetInfo[fiducialId];

    return target.timestamp > then
        || Math.abs(target.yaw) > PHOTON_VISION.MAX_ANGLE
        || Math.abs(target.pitch) > PHOTON_VISION.MAX_ANGLE;
  }

  /**
   * @return Whether or not the driver mode on the camera is active. Returns null if the camera is
   *     not connected.
   */
  public boolean isDriverModeActive() {
    return isConnected() ? m_camera.getDriverMode() : null;
  }

  public void setDriverModeActive() {
    m_camera.setDriverMode(true);
  }

  public void toggleDriverMode() {
    m_camera.setDriverMode(!m_camera.getDriverMode());
  }

  public void setDriverModeDisabled() {
    m_camera.setDriverMode(false);
  }

  public void setPipeline(int index) {
    if (m_camera.getPipelineIndex() != index) {
      m_camera.setPipelineIndex(index);
    }
  }

  public int getPipeline() {
    return m_camera.getPipelineIndex();
  }

  /**
   * Gets what PhotonVision said the best target was last time it looked.
   *
   * @return The fiducial id of the best target
   */
  public int getBestTargetFiducialId() {
    return m_bestTargetFiducialId;
  }

  /**
   * @param fiducialId The fiducial ID of the target to get the yaw of.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the desired targets yaw, will be NaN if the cached data was invalid.
   */
  public double getTargetYaw(int fiducialId, long timeoutMs) {
    if (isValidTarget(fiducialId, timeoutMs)) {
      return m_targetInfo[fiducialId].yaw;
    }
    return Double.NaN;
  }

  /**
   * @param fiducialIds The list of fiducial IDs to check.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the yaw of the first id in the list, or NaN if none are valid.
   */
  public double getTargetYaw(int[] fiducialIds, long timeoutMs) {
    for (int id : fiducialIds) {
      double yaw = getTargetYaw(id, timeoutMs);
      if (!Double.isNaN(yaw)) {
        return yaw;
      }
    }
    return Double.NaN;
  }

  /**
   * @param id The ID of the target to get the pitch of.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the desired targets pitch, will be NaN if the cached data was invalid.
   */
  public double getTargetPitch(int fiducialId, long timeoutMs) {
    if (isValidTarget(fiducialId, timeoutMs)) {
      return m_targetInfo[fiducialId].pitch;
    }
    return Double.NaN;
  }

  /**
   * @param fiducialIds The list of fiducial IDs to check.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the pitch of the first id in the list, or NaN if none are valid.
   */
  public double getTargetPitch(int[] fiducialIds, long timeoutMs) {
    for (int id : fiducialIds) {
      double pitch = getTargetPitch(id, timeoutMs);
      if (!Double.isNaN(pitch)) {
        return pitch;
      }
    }
    return Double.NaN;
  }

  /**
   * Gets an estimated pose from the subsystems pose estimator. Should only be used if the camera
   * does not see more than 1 april tag, if it does, use getPNPResult instead, as it is more
   * accurate.
   *
   * @return The robots estimated pose, if it has any april tag targets. Returns null if there are
   *     no targets.
   */
  public EstimatedRobotPose getEstimatedPose() {
    Optional<EstimatedRobotPose> estimatedPose = m_poseEstimator.update(m_result);
    return estimatedPose.isPresent() ? estimatedPose.get() : null;
  }

  /**
   * Gets the information of the multiple tag pose estimate if it exists. If the camera does not see
   * more than 1 april tag, this will return null.
   *
   * @return The PNPResult for you to get information from.
   */
  public PNPResult getPNPResult() {
    PNPResult PNPEstimate = m_result.getMultiTagResult().estimatedPose;
    return PNPEstimate.isPresent ? PNPEstimate : null;
  }

  /**
   * @param result The PNPResult to take the pose from.
   * @return The Pose3d constructed from the PNPResult.
   */
  public Pose3d pose3dFromPNPResult(PNPResult result) {
    return new Pose3d(result.best.getTranslation(), result.best.getRotation());
  }

  /**
   * @param result The PNPResult to take the pose from.
   * @return The Pose2d constructed from the PNPResult.
   */
  public Pose2d pose2dFromPNPResult(PNPResult result) {
    return new Pose2d(
        result.best.getTranslation().toTranslation2d(), result.best.getRotation().toRotation2d());
  }
}
