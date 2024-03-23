package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.POSE_FILTER;
import frc.robot.Robot;
import frc.robot.util.Utility;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class UpdatePoseWithVision extends Command {
  private int m_startingPipeline;
  private boolean m_gotPose;

  public UpdatePoseWithVision() {}

  @Override
  public void initialize() {
    m_startingPipeline = Robot.shooterCam.getPipeline();
    Robot.shooterCam.setPoseEstimationPipeline();
    m_gotPose = false;
  }

  @Override
  public void execute() {
    Optional<EstimatedRobotPose> estimatedPose = Robot.shooterCam.getEstimatedPose();

    if (estimatedPose.isEmpty()) return;

    Pose2d visionPose = estimatedPose.get().estimatedPose.toPose2d();
    Pose2d robotPose = Robot.swerve.getState().Pose;
    if (isPoseValid(visionPose, robotPose, robotPose.getRotation().getDegrees())) {

      Robot.swerve.addVisionMeasurement(visionPose, estimatedPose.get().timestampSeconds);
      m_gotPose = true;
    }
  }

  @Override
  public boolean isFinished() {
    return m_gotPose;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intakeCam.setPipeline(m_startingPipeline);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  /**
   * @param visionPose The pose from the camera
   * @param targetRotationDegrees The rotation to filter agains
   * @return Returns true if the pose is valid
   */
  public static boolean isRotationValid(Pose2d visionPose, double targetRotationDegrees) {
    return (visionPose == null
        || Utility.isWithinTolerance(
            visionPose.getRotation().getDegrees(),
            targetRotationDegrees,
            POSE_FILTER.VISION_TOLERANCE_DEGREES));
  }

  /**
   * @param visionPose The pose from the camera
   * @param targetPose The x,y to filter against
   * @param targetRotationDegrees The rotation to filter agains
   */
  public static boolean isPoseValid(
      Pose2d visionPose, Pose2d targetPose, double targetRotationDegrees) {
    if (!isRotationValid(visionPose, targetRotationDegrees)) {
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
