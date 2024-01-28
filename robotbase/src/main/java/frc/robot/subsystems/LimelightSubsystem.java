/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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
import frc.robot.Constants;

/** Controls the limelight camera options. */
public class LimelightSubsystem extends SubsystemBase {
  protected NetworkTable m_table;

  private DoublePublisher m_streamPub;
  private DoublePublisher m_pipelinePub;
  private DoubleSubscriber m_pipelineSub;
  private DoubleSubscriber m_TvSub;
  private DoubleSubscriber m_TxSub;
  private DoubleSubscriber m_TySub;
  private DoubleSubscriber m_TaSub;
  private DoubleSubscriber m_TsSub;
  private DoubleSubscriber m_ThorSub;
  private DoubleSubscriber m_TvertSub;
  private IntegerSubscriber m_Tid;
  private DoubleArraySubscriber m_botposeWpiRed;
  private DoubleArraySubscriber m_botposeWpiBlue;

  private DoubleArraySubscriber m_limelightPoseInfoSub;

  private int m_poseListenerHandle;

  /**
   * Sets the camera stream.
   *
   * @param limelightName Name of the desired limelight's shuffleboard table
   */
  public LimelightSubsystem(String limelightName) {
    m_table = NetworkTableInstance.getDefault().getTable(limelightName);

    m_streamPub = m_table.getDoubleTopic("stream").publish();
    m_pipelinePub = m_table.getDoubleTopic("pipeline").publish();
    m_pipelineSub = m_table.getDoubleTopic("pipeline").subscribe(Double.NaN);
    m_TvSub = m_table.getDoubleTopic("tv").subscribe(Constants.SHOOTER_LIMELIGHT.DEFAULT_RETURN_VALUE);
    m_TxSub = m_table.getDoubleTopic("tx").subscribe(Constants.SHOOTER_LIMELIGHT.DEFAULT_RETURN_VALUE);
    m_TySub = m_table.getDoubleTopic("ty").subscribe(Constants.SHOOTER_LIMELIGHT.DEFAULT_RETURN_VALUE);
    m_TaSub = m_table.getDoubleTopic("ta").subscribe(Constants.SHOOTER_LIMELIGHT.DEFAULT_RETURN_VALUE);
    m_TsSub = m_table.getDoubleTopic("ts").subscribe(Constants.SHOOTER_LIMELIGHT.DEFAULT_RETURN_VALUE);
    m_ThorSub = m_table.getDoubleTopic("thor").subscribe(Constants.SHOOTER_LIMELIGHT.DEFAULT_RETURN_VALUE);
    m_TvertSub = m_table.getDoubleTopic("tvert").subscribe(Constants.SHOOTER_LIMELIGHT.DEFAULT_RETURN_VALUE);
    m_Tid = m_table.getIntegerTopic("tid").subscribe(-1);

    m_botposeWpiRed = m_table
        .getDoubleArrayTopic("botpose_wpired")
        .subscribe(null, PubSubOption.keepDuplicates(true));
    m_botposeWpiBlue = m_table
        .getDoubleArrayTopic("botpose_wpiblue")
        .subscribe(null, PubSubOption.keepDuplicates(true));

    DoubleArrayTopic limelightPoseInfo = m_table.getDoubleArrayTopic("botpose");
    m_limelightPoseInfoSub = limelightPoseInfo.subscribe(null, PubSubOption.keepDuplicates(true));
  }

  public void configure() {
    setHumanPipelineActive();
    setStream(Constants.SHOOTER_LIMELIGHT.IS_PRIMARY_STREAM);
  }

  public boolean validTargetExists() {
    return 0 < getTV();
  }

  /**
   * @param id Id of the desired april tag
   * @return If the limelight sees the april tag
   */
  public boolean validAprilTagTargetExists(int id) {
    return id == m_Tid.get();
  }

  public boolean isHumanPipelineActive() {
    return getPipeline() == Constants.SHOOTER_LIMELIGHT.HUMAN_PIPELINE_INDEX;
  }

  public void setPipeline(int index) {
    m_pipelinePub.set(index);
  }

  public void setHumanPipelineActive() {
    m_pipelinePub.set(Constants.SHOOTER_LIMELIGHT.HUMAN_PIPELINE_INDEX);
  }

  public int getPipeline() {
    double value = m_pipelineSub.get();
    return (int) Math.round(value);
  }

  public void setStream(boolean isLimelightPrimary) {
    m_streamPub.set(isLimelightPrimary ? 1 : 2);
  }

  /**
   * Whether the camera has a valid target
   *
   * @return 1 for true, 0 for false
   */
  public double getTV() {
    return m_TvSub.get();
  }

  /** Horizontal offset from crosshair to target (degrees) */
  public double getTX() {
    return m_TxSub.get();
  }

  /** Vertical offset from crosshair to target (degrees) */
  public double getTY() {
    return m_TySub.get();
  }

  /** Percent of image covered by target [0, 100] */
  public double getTA() {
    return m_TaSub.get();
  }

  /** Skew or rotation (degrees, [-90, 0]) */
  public double getTS() {
    return m_TsSub.get();
  }

  /** Horizontal sidelength of rough bounding box (0 - 320 pixels) */
  public double getTHOR() {
    return m_ThorSub.get();
  }

  /** Vertical sidelength of rough bounding box (0 - 320 pixels) */
  public double getTVERT() {
    return m_TvertSub.get();
  }

  /**
   * Skew of target in degrees. Positive values are to the left, negative to the
   * right
   */
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
    double factor = proportion
        * Constants.SHOOTER_LIMELIGHT.TARGET_HEIGHT
        / Constants.SHOOTER_LIMELIGHT.TARGET_WIDTH;
    return 90.0 * (1 - factor);
  }

  public double getInchesFromTarget() {
    if (!validTargetExists()) {
      return Double.NaN;
    }

    double angleDegrees = Math.abs(getTY()) + Constants.SHOOTER_LIMELIGHT.MOUNTING_ANGLE_DEGREES;

    double heightDifference = Constants.SHOOTER_LIMELIGHT.MOUNTING_HEIGHT_INCHES
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
    return botposeToPose2d(m_botposeWpiRed.get());
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

  /*
   * @Override
   * public void periodic() {
   * SmartDashboard.putNumber("Y", getTY());
   * }
   */
}
