package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionMeasurement {
  private Pose2d m_pose;
  private double m_timestamp;

  public VisionMeasurement(Pose2d pose, double timestamp) {
    m_pose = pose;
    m_timestamp = timestamp;
  }

  public void setPose(Pose2d pose) {
    m_pose = pose;
  }

  public void setTimestamp(double timestamp) {
    m_timestamp = timestamp;
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public double getTimestamp() {
    return m_timestamp;
  }
}
