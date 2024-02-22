package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PHOTON_VISION;
import frc.robot.util.VisionMeasurement;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/** Controls the photon vision camera options. */
public class PhotonVisionCamera extends SubsystemBase {

  // all of these are protected so we can use them in the extended classes
  // which are only extended so we can control which pipelines we are using.
  protected PhotonCamera m_camera;
  protected PhotonPipelineResult m_result;
  protected List<PhotonTrackedTarget> m_targets;
  protected PhotonTrackedTarget m_bestTarget;
  protected PhotonPoseEstimator m_poseEstimator;
  protected final Transform3d ROBOT_TO_CAMERA_TRANSFORM; // if this changes, we have bigger issues.
  protected final double HEAD_ON_TOLERANCE;
  protected static boolean m_connectionLost;

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
      String cameraName, Transform3d robotToCameraTransform, double headOnTolerance) {
    m_camera = new PhotonCamera(cameraName);
    ROBOT_TO_CAMERA_TRANSFORM = robotToCameraTransform;
    HEAD_ON_TOLERANCE = headOnTolerance;
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
      m_targets = m_result.getTargets();
      m_bestTarget = m_result.getBestTarget();

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
   * @return Whether the camera has a valid target and is connected
   */
  public boolean validTargetExists() {
    return getTV();
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
   * @return Whether the camera has a valid target and is connected
   */
  public boolean getTV() {
    return isConnected() && m_result.hasTargets();
  }

  /**
   * @return Horizontal offset from crosshair to target (degrees)
   */
  public double getTX() {
    return getTV() ? m_bestTarget.getYaw() : Double.NaN;
  }

  /**
   * Gets the best targets detected bounding box horizontal distance, uses unknown units, assume
   * pixels. TAKE NOTE!
   *
   * <pre>
   * ⟶  +X  3 ----- 2
   * |      |       |
   * V      |       |
   * +Y     0 ----- 1
   * </pre>
   *
   * X and Y increase opposite usual ways. Use accordingly.
   *
   * @return The best targets detected bounding box horizontal side length in what are are assumed
   *     to be pixels. Returns NaN if the camera has no targets or is disconnected.
   */
  public double getHorizontalTargetLength() {
    if (!getTV()) {
      return Double.NaN;
    }
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
   * +Y     0 ----- 1
   * </pre>
   *
   * X and Y increase opposite usual ways. Use accordingly.
   *
   * @return The best targets detected bounding box vertical side length in what are are assumed to
   *     be pixels. Returns NaN if the camera has no targets or is disconnected.
   */
  public double getVerticalTargetLength() {
    if (!getTV()) {
      return Double.NaN;
    }
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
    return getTV() ? m_bestTarget.getYaw() : Double.NaN;
  }

  /** Vertical offset from crosshair to target (degrees) */
  public double getTY() {
    return getTV() ? m_bestTarget.getPitch() : Double.NaN;
  }

  /**
   * Gets the best targets pitch from the crosshair to the target
   *
   * @return Best targets pitch from crosshair to target
   */
  public double getPitch() {
    return getTV() ? m_bestTarget.getPitch() : Double.NaN;
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
    return getTV() ? m_bestTarget.getArea() : Double.NaN;
  }

  /** Skew or rotation (degrees, [-90, 0]) */
  public double getTS() {
    return getTV() ? m_bestTarget.getSkew() : Double.NaN;
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
   * @return The robots estimated pose, if it has any april tag targets. Returns null if there are
   *     no targets.
   */
  public VisionMeasurement getEstimatedPose() {
    Optional<EstimatedRobotPose> optionalPose = m_poseEstimator.update();

    if (!optionalPose.isPresent()) {
      return null;
    }

    Pose3d pose3d = optionalPose.get().estimatedPose;

    Pose2d pose =
        new Pose2d(pose3d.getX(), pose3d.getY(), new Rotation2d(pose3d.getRotation().getZ()));

    return new VisionMeasurement(pose, optionalPose.get().timestampSeconds);
  }

  /**
   * @return The best targets fiducial ID, returns -1 if it doesnt have one, and -2 if there are no
   *     targets
   */
  public int getLastTargetID() {
    return getTV() ? m_bestTarget.getFiducialId() : -2;
  }

  public List<PhotonTrackedTarget> filterAprilTags(int[] tagsToFilterFor) {
    List<PhotonTrackedTarget> filteredTargets = m_targets;
    for (int i = 0; i < m_targets.size(); i++) {
      boolean removeTag = true;
      for (int targetIDWanted : tagsToFilterFor) {
        if (filteredTargets.get(i).getFiducialId() == targetIDWanted) {
          removeTag = false;
        }
      }
      if (removeTag) {
        filteredTargets.remove(i);
      }
    }
    return filteredTargets;
  }
}
