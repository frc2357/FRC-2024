/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PHOTON_VISION;
import frc.robot.Constants;

/** Controls the limelight camera options. */
public class PhotonVision extends SubsystemBase {

  protected PhotonCamera m_camera;
  private PhotonPipelineResult m_result;
  private List<PhotonTrackedTarget> m_targets;
  private PhotonPoseEstimator m_poseEstimator;
  /**
   * Sets the camera stream.
   *
   * @param cameraName Name of the cameras Photon Vision network table. MUST match the net tables name, or it wont work.
   */
  public PhotonVision(String cameraName) {
    m_camera = new PhotonCamera(cameraName);
  }

  public void configure() {
    setHumanPipelineActive();
    m_poseEstimator = new PhotonPoseEstimator(PHOTON_VISION.APRIL_TAG_FIELD_LAYOUT, PHOTON_VISION.POSE_STRATEGY, m_camera, PHOTON_VISION.ROBOT_TO_CAMERA_TRANSFORM);
    setStream(Constants.SHOOTER_LIMELIGHT.IS_PRIMARY_STREAM);
  }

  private void getResult(){
    m_result = m_camera.getLatestResult();
    m_targets = m_result.getTargets();
  }

  public boolean validTargetExists() {
    return getTV();
  }

  
  /**
   * @return The current pipelines latency in milliseconds
   */
  public double getLatencyMillis(){
    return m_result.getLatencyMillis();
  }

  /**
   * @param id Id of the desired april tag
   * @return If the limelight sees the april tag
   */
  public boolean validAprilTagTargetExists(int id) {
    for(PhotonTrackedTarget target : m_targets){
      if(target.getFiducialId() != -1){
        return true;
      }
    }
    return false;
  }

  public boolean isHumanPipelineActive() {
    return getPipeline() == Constants.SHOOTER_LIMELIGHT.HUMAN_PIPELINE_INDEX;
  }

  public void setPipeline(int index) {
    m_camera.setDriverMode(false);
    m_camera.setPipelineIndex(index);
  }

  public void setHumanPipelineActive() {
    m_camera.setDriverMode(true);
  }

  public int getPipeline() {
    return m_camera.getPipelineIndex();
  }

  public void setStream(boolean isLimelightPrimary) {
    //TODO: Figure out if this has an equal in photon vision
    //m_streamPub.set(isLimelightPrimary ? 1 : 2);
  }

  /**
   * Whether the camera has a valid target
   */
  public boolean getTV() {
    
    return m_result.hasTargets();
  }

  /** Horizontal offset from crosshair to target (degrees) */
  public double getTX() {
    return m_targets.get(0).getYaw();
  }

  /** Vertical offset from crosshair to target (degrees) */
  public double getTY() {
    return m_targets.get(0).getPitch();
  }

  /** Percent of image covered by target [0, 100] */
  public double getTA() {
    return m_targets.get(0).getArea();
  }

  /** Skew or rotation (degrees, [-90, 0]) */
  public double getTS() {
    return m_targets.get(0).getSkew();
  }

  /** Horizontal sidelength of rough bounding box (0 - 320 pixels) */
  public double getTHOR() {
    return m_ThorSub.get();
  }

  /** Vertical sidelength of rough bounding box (0 - 320 pixels) */
  public double getTVERT() {
    return m_TvertSub.get();
  }

  /** Skew of target in degrees. Positive values are to the left, negative to the right */
  public double getSkew() {
    if (!validTargetExists()) {
      return Double.NaN;
    }

    double ts = getTS();
    if (ts < -45) {
      return ts + 90;
    } else {
      return ts;
    }
  }

  public boolean isHeadOn() {
    if (!validTargetExists()) {
      return false;
    }

    double skew = getSkew();
    return (Constants.SHOOTER_LIMELIGHT.HEAD_ON_TOLERANCE <= skew
        && skew <= Constants.SHOOTER_LIMELIGHT.HEAD_ON_TOLERANCE);
  }

  public boolean isToLeft() {
    if (!validTargetExists()) {
      return false;
    }

    return getSkew() > Constants.SHOOTER_LIMELIGHT.HEAD_ON_TOLERANCE;
  }

  public boolean isToRight() {
    if (!validTargetExists()) {
      return false;
    }

    return getSkew() < Constants.SHOOTER_LIMELIGHT.HEAD_ON_TOLERANCE;
  }

  public double getTargetRotationDegrees() {
    if (!validTargetExists()) {
      return Double.NaN;
    }

    if (isHeadOn()) {
      return 0.0;
    } else if (isToLeft()) {
      return -getRotationAngle();
    } else {
      return getRotationAngle();
    }
  }

  private double getRotationAngle() {
    if (!validTargetExists()) {
      return Double.NaN;
    }

    double proportion = getTHOR() / getTVERT();
    double factor =
        proportion
            * Constants.SHOOTER_LIMELIGHT.TARGET_HEIGHT
            / Constants.SHOOTER_LIMELIGHT.TARGET_WIDTH;
    return 90.0 * (1 - factor);
  }

  public double getInchesFromTarget() {
    if (!validTargetExists()) {
      return Double.NaN;
    }

    double angleDegrees = Math.abs(getTY()) + Constants.SHOOTER_LIMELIGHT.MOUNTING_ANGLE_DEGREES;

    double heightDifference =
        Constants.SHOOTER_LIMELIGHT.MOUNTING_HEIGHT_INCHES
            - Constants.SHOOTER_LIMELIGHT.TARGET_HEIGHT_FROM_FLOOR;
    double distance = heightDifference / Math.tan(Math.toRadians(angleDegrees));

    return distance;
  }

  public Pose2d getLimelightPose2d() {
    return botposeToPose2d(m_limelightPoseInfoSub.get());
  }

  public Long getLastTargetID() {
    return m_Tid.get();
  }

  public Pose2d getRedPose() {
    return m_camera.get
  }

  public Pose2d getBluePose() {
    return botposeToPose2d(m_botposeWpiBlue.get());
  }

  public double getBlueBotposeTimestamp() {
    return calculateTimestamp(m_botposeWpiBlue.get());
  }

  public double getRedBotposeTimestamp() {
    return calculateTimestamp(m_botposeWpiRed.get());
  }

  public double calculateTimestamp(double[] botpose) {
    if (botpose == null) {
      return 0;
    }

    return Timer.getFPGATimestamp() - (botpose[6] / 1000);
  }

  public static Pose2d botposeToPose2d(double[] botpose) {
    if (botpose == null) {
      return null;
    }

    Translation2d t2d = new Translation2d(botpose[0], botpose[1]);
    Rotation2d r2d = Rotation2d.fromDegrees(botpose[5]);
    return new Pose2d(t2d, r2d);
  }

  @Override
  public void periodic() {
    getResult();
  }
}
