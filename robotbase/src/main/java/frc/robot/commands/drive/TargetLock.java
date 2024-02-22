package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.state.RobotState.DriveControlState;
import frc.robot.subsystems.PhotonVisionCamera;

public class TargetLock extends Command {
  private int m_pipelineIndex;
  private PhotonVisionCamera m_camera;
  private int m_deafaultPipeline;

  public TargetLock(int pipelineIndex, PhotonVisionCamera camera) {
    m_pipelineIndex = pipelineIndex;
    m_camera = camera;
  }

  @Override
  public void initialize() {
    Robot.state.setDriveControlState(DriveControlState.TARGET_LOCK);
    m_camera.setPipeline(m_pipelineIndex);
    SWERVE.TARGET_LOCK_ROTATION_PID_CONTROLLER.reset();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.state.setDriveControlState(DriveControlState.FIELD_RELATIVE);
    m_camera.setPipeline(m_deafaultPipeline);
    SWERVE.TARGET_LOCK_ROTATION_PID_CONTROLLER.setP(SWERVE.TARGET_LOCK_ROTATION_KP);
  }
}
