package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.POSE_FILTER;

public class PoseHelpers {

  /**
   * @param visionPose The pose from the camera
   * @param targetRotationDegrees The rotation to filter agains
   * @return Returns true if the pose is valid
   */
  public static boolean isPoseValid(Pose2d visionPose, double targetRotationDegrees) {
    return (visionPose == null
        || Utility.isWithinTolerance(
            visionPose.getRotation().getDegrees(),
            targetRotationDegrees,
            POSE_FILTER.VISION_TOLERANCE_DEGREES));
  }

  /**
   * @param visionPose The pose from the camera
   * @param targetPose The pose to filter against
   * @param targetRotationDegrees The rotation to filter agains
   */
  public static boolean isPoseValid(
      Pose2d visionPose, Pose2d targetPose, double targetRotationDegrees) {
    if (!isPoseValid(visionPose, targetRotationDegrees)) {
      return false;
    }

    if (targetPose != null
        && Utility.isWithinTolerance(
            visionPose.getX(), targetPose.getX(), POSE_FILTER.VISION_TOLERANCE_METERS)
        && Utility.isWithinTolerance(
            visionPose.getY(), targetPose.getY(), POSE_FILTER.VISION_TOLERANCE_METERS)) {
      return true;
    }
    return false;
  }
}
