/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.PHOTON_VISION;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/** Controls the limelight camera options. */
public class PhotonVision extends SubsystemBase {

  //all of these are protected so we can use them in the extended classes
  //which are only extended so we can control which pipelines we are using.
  protected PhotonCamera m_camera;
  protected PhotonPipelineResult m_result;
  protected List<PhotonTrackedTarget> m_targets;
  protected PhotonTrackedTarget m_bestTarget;
  protected PhotonPoseEstimator m_poseEstimator;
  protected final Transform3d m_ROBOT_TO_CAMERA_TRANSFORM; //if this changes, we have bigger issues.
  protected final double m_HEAD_ON_TOLERANCE;

  /**
   * Sets the camera stream.
   *
   * @param cameraName Name of the cameras Photon Vision network table. MUST match the net tables
   *     name, or it wont work.
   */
  public PhotonVision(String cameraName, Transform3d robotToCameraTransform, double headOnTolerance) {
    m_camera = new PhotonCamera(cameraName);
    m_ROBOT_TO_CAMERA_TRANSFORM = robotToCameraTransform;
    m_HEAD_ON_TOLERANCE = headOnTolerance;
  }

  public void configure() {
    setDriverModeActive();
    m_poseEstimator =
        new PhotonPoseEstimator(
            PHOTON_VISION.APRIL_TAG_FIELD_LAYOUT,
            PHOTON_VISION.POSE_STRATEGY,
            m_camera,
            m_ROBOT_TO_CAMERA_TRANSFORM);
  }

  /**
   * Fetches the latest pipeline result.<p>
   * YOU SHOULD NEVER CALL THIS! This is for the Robot periodic ONLY. NEVER call this method outside of it.s
   */
  public void fetchResult(){
    m_result = m_camera.getLatestResult();
    m_targets = m_result.getTargets();
    m_bestTarget = m_result.getBestTarget();
  }

  public boolean validTargetExists() {
    return getTV();
  }

  /**
   * @return The current pipelines latency in milliseconds
   */
  public double getLatencyMillis() {
    return m_result.getLatencyMillis();
  }

  /**
   * @param id Id of the desired april tag
   * @return If the limelight sees the april tag
   */
  public boolean validAprilTagTargetExists(int id) {
    for (PhotonTrackedTarget target : m_targets) {
      if (target.getFiducialId() == id) {
        return true;
      }
    }
    return false;
  }

  public boolean isHumanPipelineActive() {
    return m_camera.getDriverMode();
  }

  public void setDriverModeActive() {
    m_camera.setDriverMode(true);
  }

  public void toggleDriverMode(){
    m_camera.setDriverMode(!m_camera.getDriverMode());
  }

  public void setDriverModeDisabled(){
    m_camera.setDriverMode(false);
  }

  public void setPipeline(int index) {
    m_camera.setDriverMode(false);
    m_camera.setPipelineIndex(index);
  }

  

  public int getPipeline() {
    return m_camera.getPipelineIndex();
  }

  /** Whether the camera has a valid target */
  public boolean getTV() {
    return m_result.hasTargets();
  }

  /** Horizontal offset from crosshair to target (degrees) */
  public double getTX() {
    return m_bestTarget.getYaw();
  }

  /**
   * Gets the best targets detected bounding box horizontal distance, uses unknown units, assume
   * pixels. TAKE NOTE!
   *
   * <pre>
   * ⟶  +X  3 ----- 2
   * |      |       |
   * V      |       |
   * +Y     0 ----- 1</pre>
   *
   * X and Y increase opposite usual ways. Use accordingly.
   *
   * @return The best targets detected bounding box horizontal side length in what are are assumed
   *     to be pixels.
   */
  public double getHorizontalTargetLength() {
    List<TargetCorner> corners = m_bestTarget.getDetectedCorners();
    TargetCorner originCorner = corners.get(0); // basing off of known fiducial corner order
    TargetCorner bottomRightCorner = corners.get(1);
    for (TargetCorner corner : corners) {
      if (corner.x < originCorner.x && corner.y > originCorner.y) {
        originCorner = corner;
      }
      if (corner.x > bottomRightCorner.x && corner.y > bottomRightCorner.y) {
        bottomRightCorner = corner;
      }
    }
    return bottomRightCorner.x - originCorner.x;
  }

  /**
   * Gets the best targets detected bounding box vertical distance, uses unknown units, assume
   * pixels. TAKE NOTE!
   *
   * <pre>
   * ⟶  +X  3 ----- 2
   * |      |       |
   * V      |       |
   * +Y     0 ----- 1</pre>
   *
   * X and Y increase opposite usual ways. Use accordingly.
   *
   * @return The best targets detected bounding box vertical side length in what are are assumed to
   *     be pixels.
   */
  public double getVerticalTargetLength() {
    List<TargetCorner> corners = m_bestTarget.getDetectedCorners();
    TargetCorner originCorner = corners.get(0); // basing off of known fiducial corner order
    TargetCorner bottomRightCorner = corners.get(1);
    for (TargetCorner corner : corners) {
      if (corner.x < originCorner.x && corner.y > originCorner.y) {
        originCorner = corner;
      }
      if (corner.x > bottomRightCorner.x && corner.y > bottomRightCorner.y) {
        bottomRightCorner = corner;
      }
    }
    return bottomRightCorner.y - originCorner.y;
  }

  /**
   * Gets the best targets yaw from the crosshair to the target
   *
   * @return Best targets yaw from crosshair to target
   */
  public double getYaw() {
    return m_bestTarget.getYaw();
  }

  /** Vertical offset from crosshair to target (degrees) */
  public double getTY() {
    return m_bestTarget.getPitch();
  }

  /**
   * Gets the best targets pitch from the crosshair to the target
   *
   * @return Best targets pitch from crosshair to target
   */
  public double getPitch() {
    return m_bestTarget.getPitch();
  }

  /** Percent of image covered by target [0, 100] */
  public double getTA() {
    return m_bestTarget.getArea();
  }

  /**
   * Percent of image covered by the best target [0, 100]
   *
   * @return percentage of the image that the best target covers
   */
  public double getArea() {
    return m_bestTarget.getArea();
  }

  /** Skew or rotation (degrees, [-90, 0]) */
  public double getTS() {
    return m_bestTarget.getSkew();
  }

  /** Yaw of target in degrees. Positive values are to the left, negative to the right.<p>
   * The "getSkew()" equivalent in PhotonVision.
   */
  public double getFilteredYaw() {
    if (!validTargetExists()) {
      return Double.NaN;
    }

    double yaw = getYaw();
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

    double skew = getYaw();
    return (m_HEAD_ON_TOLERANCE <= skew
        && skew <= m_HEAD_ON_TOLERANCE);
  }

  public boolean isToLeft() {
    if (!validTargetExists()) {
      return false;
    }

    return getFilteredYaw() > m_HEAD_ON_TOLERANCE;
  }

  public boolean isToRight() {
    if (!validTargetExists()) {
      return false;
    }

    return getFilteredYaw() < m_HEAD_ON_TOLERANCE;
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

  public int getLastTargetID() {
    return m_bestTarget.getFiducialId();
  }

  public List<PhotonTrackedTarget> filterAprilTags(int[] tagsToFilterFor) {
    List<PhotonTrackedTarget> filteredTargets = m_targets;
    for (int i = 0; i < m_targets.size(); i++) {
      boolean removeTag = true;
      for(int targetIDWanted : tagsToFilterFor){
        if(filteredTargets.get(i).getFiducialId() == targetIDWanted){
          removeTag = false;
        }
      }
      if(removeTag){
        filteredTargets.remove(i);
      }
    }
    return filteredTargets;
  }

}
