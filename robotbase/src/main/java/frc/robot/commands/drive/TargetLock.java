package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.state.RobotState.DriveControlState;

public class TargetLock extends Command {
  private int m_pipelineIndex;

  public TargetLock(int pipelineIndex) {
    m_pipelineIndex = pipelineIndex;
  }

  @Override
  public void initialize() {
    Robot.state.setDriveControlState(DriveControlState.TARGET_LOCK);
    Robot.shooterPhotonCamera.setPipeline(m_pipelineIndex);
    SWERVE.ROTATION_PID_CONTROLLER.reset();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.state.setDriveControlState(DriveControlState.FIELD_RELATIVE);
    Robot.shooterPhotonCamera.setHumanPipelineActive();
    SWERVE.ROTATION_PID_CONTROLLER.setP(SWERVE.ROTATION_KP);
  }
}
