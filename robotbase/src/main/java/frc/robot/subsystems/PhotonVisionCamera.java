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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;

/** Controls the photon vision camera options. */
public class PhotonVisionCamera extends SubsystemBase {

  /*
   * The class for the object we use to cache our target data
   */
  private static class TargetInfo {
    public double yaw = Double.NaN;
    public double pitch = Double.NaN;
    public long timestamp = 0;
  }

  // all of these are protected so we can use them in the extended classes
  // which are only extended so we can control which pipelines we are using.

  /** The actual camera object that we get everything from. */
  protected PhotonCamera m_camera;

  /** The result we fetch from PhotonLib each loop. */
  protected PhotonPipelineResult m_result;

  /** The list of results that we have to get each time. */
  private List<PhotonPipelineResult> m_resultList;

  /**
   * The list of TargetInfo objects where we cache all of the target data.
   *
   * <p>Index 0 is the best gamepeice that we detect.
   *
   * <p>Index 1-16 are the AprilTags that are on the field.
   */
  protected final TargetInfo[] m_targetInfo;

  /**
   * The pose estimator for the subsystem TODO: see if we want to change this, and do whatever makes
   * ISAM work.
   */
  protected PhotonPoseEstimator m_poseEstimator;

  /** The robot origin to camera lens transform 3D that we use to make the pose estimator. */
  protected final Transform3d ROBOT_TO_CAMERA_TRANSFORM; // if this changes, we have bigger issues.

  /** Whether or not we have connection with the camera still */
  protected boolean m_connectionLost;

  /**
   * The fiducial ID of the best target we have.
   *
   * <p>Used for methods that dont take in a fid ID but do some april tag stuff.
   */
  protected int m_bestTargetFiducialId;

  /**
   * Represents a camera from PhotonVision.
   *
   * <p>Handles connection, caching, calculating stuff, filtering, mostly everything.
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

    // index 0 is for note detection, 1-16 correspond to apriltag fiducial IDs
    m_targetInfo = new TargetInfo[17];
    for (int i = 0; i < m_targetInfo.length; i++) {
      m_targetInfo[i] = new TargetInfo();
    }
  }

  /** Sets up everything about the class that is not done in the constructor. */
  public void configure() {
    m_poseEstimator =
        new PhotonPoseEstimator(
            PHOTON_VISION.APRIL_TAG_FIELD_LAYOUT,
            PHOTON_VISION.POSE_STRATEGY,
            ROBOT_TO_CAMERA_TRANSFORM);
  }

  /**
   * Fetches the latest pipeline result.
   *
   * <p>
   *
   * <h1>YOU SHOULD NEVER CALL THIS! This is for the Robot periodic ONLY. NEVER call this method
   * outside of it. </h1>
   */
  public void updateResult() {
    if (!m_camera.isConnected()) {
        if(!m_connectionLost){
            m_connectionLost = true;
            DriverStation.reportError(
                "[" + m_camera.getName() + "]\n" + PHOTON_VISION.LOST_CONNECTION_ERROR_MESSAGE, false);
        }
      return;
    }

    m_resultList = m_camera.getAllUnreadResults();
    int finalResultIndex = m_resultList.size() - 1;
    m_result = m_resultList.get(finalResultIndex);

    if (m_result == null || !m_result.hasTargets()) {
      return;
    }
    m_bestTargetFiducialId = m_result.getBestTarget().getFiducialId();
    if (m_bestTargetFiducialId == -1) {
      // the fact that the best tarets ID is -1 means its a gamepeice
      // so we cache them differently.
      cacheForGamepeices(m_result.targets);
    } else {
      cacheForAprilTags(m_result.targets);
    }
    if (m_connectionLost) {
      m_connectionLost = false;
      DriverStation.reportWarning(
          "[" + m_camera.getName() + "]\n" + PHOTON_VISION.CONNECTION_REGAINED_NOFICATION_MESSAGE,
          false);
    }
  }

  /**
   * The method to cache target data for gamepeices.
   *
   * @param targetList The list of targets that it pulls data from to cache.
   */
  private void cacheForGamepeices(List<PhotonTrackedTarget> targetList) {
    long now = System.currentTimeMillis();
    PhotonTrackedTarget bestTarget = calculateBestGamepeiceTarget(targetList);
    TargetInfo targetInfo = m_targetInfo[0];
    targetInfo.yaw = bestTarget.getYaw();
    targetInfo.pitch = bestTarget.getPitch();
    targetInfo.timestamp = now;
  }

  /**
   * The method to cache target data for AprilTags.
   *
   * @param targetList The list of targets that it pulls data from to cache.
   */
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

  /**
   * Calculates the best gamepeice in a list of PhotonTrackedTargets.
   *
   * <p>This is made to sort through gamepeices if they are next to eachother.
   *
   * @param targetList List of the targets to sort through.
   * @return The target that is in a acceptable pitch range, and is the most centered.
   */
  public PhotonTrackedTarget calculateBestGamepeiceTarget(List<PhotonTrackedTarget> targetList) {
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
   * @return The timestamp of the latest pipeline result in seconds. Returns Double.NaN if the
   *     camera is not connected.
   */
  public double getTimestampSeconds() {
    return isConnected() ? m_result.getTimestampSeconds() : Double.NaN;
  }

  /**
   * Compares the current system time to the last cached timestamp, and sees if it is older than the
   * passsed in timeout.
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
   * Sets the pipeline index to make the camera go to.
   *
   * @param index The index to make it be set to.
   */
  public void setPipeline(int index) {
    if (m_camera.getPipelineIndex() != index) {
      m_camera.setPipelineIndex(index);
    }
  }

  /**
   * Gets the pipeline index that an NT subscriber returns.
   *
   * @return The returned pipeline index number.
   */
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
   * @return Returns the desired targets yaw. <strong>Will be NaN if the cached data was invalid.
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
   * @return Returns the yaw of the first id in the list, <strong>or NaN if none are valid.
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
   * @return Returns the desired targets pitch, <strong>will be NaN if the cached data was invalid.
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
   * @return Returns the pitch of the first id in the list, <strong>or NaN if none are valid.
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
   * Gets an estimated pose from the subsystems pose estimator.
   *
   * <p>Should only be used if the camera does not see more than 1 april tag, if it does, use
   * getPnpResult instead, as it is more accurate.
   *
   * @return The robots estimated pose, if it has any april tag targets. <strong>Returns null if
   *     there are no targets.
   */
  public EstimatedRobotPose getEstimatedPose() {
    Optional<EstimatedRobotPose> estimatedPose = m_poseEstimator.update(m_result);
    return estimatedPose.isPresent() ? estimatedPose.get() : null;
  }

  /**
   * Gets the information of the multiple tag pose estimate if it exists.
   *
   * <p>If the camera does not see more than 1 april tag, <strong> this will return null. </strong>
   *
   * @return The PnpResult for you to get information from.
   */
  public PnpResult getPnpResult() {
    return m_result.multitagResult.isPresent() ? m_result.multitagResult.get().estimatedPose : null;
  }

  /**
   * @param result The PnpResult to take the pose from.
   * @return The Pose3d constructed from the PnpResult.
   */
  public Pose3d pose3dFromPnpResult(PnpResult result) {
    return new Pose3d(result.best.getTranslation(), result.best.getRotation());
  }

  /**
   * @param result The PnpResult to take the pose from.
   * @return The Pose2d constructed from the PnpResult.
   */
  public Pose2d pose2dFromPnpResult(PnpResult result) {
    return new Pose2d(
        result.best.getTranslation().toTranslation2d(), result.best.getRotation().toRotation2d());
  }

  /**
   * @return The number of targets seen.
   */
  public int numberOfTargetsSeen() {
    return m_result.targets.size();
  }

  /**
   * @return Whether or not the camera has a target.
   */
  public boolean hasTarget() {
    return m_result.hasTargets();
  }
}
