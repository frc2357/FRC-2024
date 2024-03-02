package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PHOTON_VISION;

/** Controls the photon vision camera options. */
public class PhotonVisionCamera extends SubsystemBase {

  // all of these are protected so we can use them in the extended classes
  // which are only extended so we can control which pipelines we are using.
  protected PhotonCamera m_camera;
  protected PhotonPipelineResult m_result;
  protected final double[] m_targetYawCache;
  protected final double[] m_targetPitchCache;
  protected final double[] m_targetTimeStampMillisCache;
  protected PhotonPoseEstimator m_poseEstimator;
  protected final Transform3d ROBOT_TO_CAMERA_TRANSFORM; // if this changes, we have bigger issues.
  protected final double HEAD_ON_TOLERANCE;
  protected static boolean m_connectionLost;
  protected final int LOOPS_TO_CACHE_TARGET_DATA;
  protected int m_bestTargetAdjustedId;

  /**
   * Sets the camera stream.
   *
   * @param cameraName Name of the cameras Photon Vision network table. MUST match the net tables
   *     name, or it wont work.
   * @param robotToCameraTransform The Transform3d of the robots coordinate center to the camera.
   * @param headOnTolerance The tolerance for declaring whether or not the camera is facing a target
   *     head on.
   */
  public PhotonVisionCamera(
      String cameraName, Transform3d robotToCameraTransform, double headOnTolerance, int loopsToCacheTargetData) {
    m_camera = new PhotonCamera(cameraName);
    ROBOT_TO_CAMERA_TRANSFORM = robotToCameraTransform;
    HEAD_ON_TOLERANCE = headOnTolerance;
    LOOPS_TO_CACHE_TARGET_DATA = loopsToCacheTargetData;
    m_targetYawCache = new double[17];//set to have enough slots for every april tag + 1, for the gamepeice caching.
    m_targetPitchCache = new double[17];
    m_targetTimeStampMillisCache = new double[17];
    for(double number : m_targetYawCache){
      number = Double.NaN; //setting them to NaN by default.
    }
    for(double number : m_targetPitchCache){
      number = Double.NaN; //setting them to NaN by default.
    }
    for(double number : m_targetTimeStampMillisCache){
      number = Double.NaN; //setting them to NaN by default.
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
    if (m_camera.isConnected()) {
      m_result = m_camera.getLatestResult();
      if (m_result.hasTargets()) {
        var currentTimeMillis = System.currentTimeMillis();
        List<PhotonTrackedTarget> targetList = m_result.targets;
        m_bestTargetAdjustedId = m_result.getBestTarget().getFiducialId() + 1;
        for(PhotonTrackedTarget targetSeen : targetList){
          var adjustedTargetId = targetSeen.getFiducialId()+1; //adds one to make sure that gamepeices work for caching.
          //they have a fiducial ID of -1, so adding one makes them 0. Yes, its magic.
          m_targetYawCache[adjustedTargetId] = targetSeen.getYaw();
          m_targetPitchCache[adjustedTargetId] = targetSeen.getPitch();
          m_targetTimeStampMillisCache[adjustedTargetId] = currentTimeMillis;
        }
      }

      if (m_connectionLost) {
        m_connectionLost = false;
        DriverStation.reportWarning(PHOTON_VISION.CONNECTION_REGAINED_NOFICATION_MESSAGE, false);
      }
    } else if (!m_connectionLost) {
      m_connectionLost = true;
      DriverStation.reportError(PHOTON_VISION.LOST_CONNECTION_ERROR_MESSAGE, false);
    }
  }

  /**
   * @return Whether or not the camera is connected.
   */
  public boolean isConnected() {
    return m_connectionLost; // uses this because it will be checked every loop
  }

  /**
   * @return Whether the camera has a valid target from the latest result and is connected.
   */
  public boolean validTargetExists() {
    return isConnected() && (m_result.hasTargets());
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
   * Compares the current system time to the last cached timestamp and sees if it is older than is acceptable.
   * <p> If it is older than acceptable, sets the target data to NaN. 
   * <code> isValidTarget(int id) </code> is a better option if you want to see if a target is valid, but not set it to NaN if it is invalid.
   * @param id Id of the desired target
   * @return If the camera sees the target, or a valid version of its data is cached.
   */
  public boolean validateTarget(int id) {
    int adjustedId = id + 1;
    boolean isValidTarget = isValidTarget(adjustedId);
    if(!isValidTarget){
      m_targetYawCache[adjustedId] = Double.NaN;
      m_targetPitchCache[adjustedId] = Double.NaN;
      m_targetTimeStampMillisCache[adjustedId] = Double.NaN;
    }
    return isValidTarget;
  }

  /**
   * Compares the current system time to the last cached timestamp and sees if it is older than is acceptable.<p>
   * <code> validateTarget(int id) </code> is a better option if you want to see if a target is valid, and set it to NaN if it is invalid.
   * @param id Fiducial ID of the desired target to valid the data of.
   * @return If the camera sees the target, or if a valid version of its data is cached.
   */
  public boolean isValidTarget(int id){
    return (m_targetTimeStampMillisCache[id+1] != Double.NaN ?
      (System.currentTimeMillis() - m_targetTimeStampMillisCache[id+1])/20 > 
      LOOPS_TO_CACHE_TARGET_DATA : false);
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
    m_camera.setPipelineIndex(index);
  }

  public int getPipeline() {
    return m_camera.getPipelineIndex();
  }

  /**
   * Gets the best targets yaw from the crosshair to the target
   * 
   * @return Best targets yaw from crosshair to target
   */
  public double getBestTargetYaw() {
    return validateTarget(m_bestTargetAdjustedId) ? m_targetYawCache[m_bestTargetAdjustedId] : Double.NaN;
  }

  
  /**
   * Gets the best targets pitch from the crosshair to the target
   *
   * @return Best targets pitch from crosshair to target
   */
  public double getBestTargetPitch() {
    return validateTarget(m_bestTargetAdjustedId) ? m_targetPitchCache[m_bestTargetAdjustedId] : Double.NaN;
  }

  /** Vertical offset from crosshair to target (degrees) */
  public double getBestTargetTY() {
    return getBestTargetPitch();
  }

  /**
   * @param id The ID of the target to get the yaw of.
   * @return Returns the desired targets yaw, will be NaN if the cached data was invalid.
   */
  public double getTargetYaw(int id){
    int adjustedId = id + 1;
    validateTarget(adjustedId);
    return m_targetYawCache[adjustedId];
  }

  /**
   * @param id The ID of the target to get the pitch of.
   * @return Returns the desired targets pitch, will be NaN if the cached data was invalid.
   */
  public double getTargetPitch(int id){
    int adjustedId = id + 1;
    validateTarget(adjustedId);
    return m_targetPitchCache[adjustedId];
  }

  /**
   * @param id The ID of the target to get the yaw of.
   * @return Returns the desired targets yaw, will be NaN if the cached data was invalid.
   */
  public double getTargetTimestampMillis(int id){
    int adjustedId = id + 1;
    validateTarget(adjustedId);
    return m_targetTimeStampMillisCache[adjustedId];
  }

  /**
   * Yaw of target in degrees. Positive values are to the left, negative to the right.
   *
   * <p>The "getSkew()" equivalent in PhotonVision.
   */
  public double getFilteredYaw() {
    if (!validTargetExists()) {
      return Double.NaN;
    }

    double yaw = getBestTargetYaw();
    if (yaw < -45) {
      return yaw + 90;
    } else {
      return yaw;
    }
  }

  public boolean isHeadOn() {
    if (!validTargetExists()) {
      return false;
    }

    double skew = getBestTargetYaw();
    return (HEAD_ON_TOLERANCE <= skew && skew <= HEAD_ON_TOLERANCE);
  }

  public boolean isToLeft() {
    if (!validTargetExists()) {
      return false;
    }

    return getFilteredYaw() > HEAD_ON_TOLERANCE;
  }

  public boolean isToRight() {
    if (!validTargetExists()) {
      return false;
    }

    return getFilteredYaw() < HEAD_ON_TOLERANCE;
  }

  public double getTargetRotationDegrees() {
    if (!validTargetExists()) {
      return Double.NaN;
    }

    if (isHeadOn()) {
      return 0.0;
    }
    return getFilteredYaw();
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

  /**
   * @return The best targets fiducial ID, returns -1 if it doesnt have one, and -2 if there are no
   *     targets
   */
  public int getLastTargetID() {
    return m_bestTargetAdjustedId;
  }
}
