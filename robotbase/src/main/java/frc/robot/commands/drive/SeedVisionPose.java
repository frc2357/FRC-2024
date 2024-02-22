package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VISION_PIPELINES;
import frc.robot.Robot;
import frc.robot.util.VisionMeasurement;

public class SeedVisionPose extends Command {
  public SeedVisionPose() {}

  @Override
  public void initialize() {
    Robot.shooterCam.setPipeline(VISION_PIPELINES.POSE_ESTIMATION_PIPELINE);
  }

  @Override
  public void execute() {
    VisionMeasurement visionMeasurement = Robot.shooterCam.getEstimatedPose();

    Robot.swerve.seedVisionPose(visionMeasurement.getPose());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
