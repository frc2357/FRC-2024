package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.state.RobotState.DriveControlState;

public class TargetLockCommand extends Command {
  private int m_pipelineIndex;

  public TargetLockCommand(int pipelineIndex) {
    m_pipelineIndex = pipelineIndex;
  }

  @Override
  public void initialize() {
    Robot.state.setDriveControlState(DriveControlState.TARGET_LOCK);
    Robot.shooterLimelight.setPipeline(m_pipelineIndex);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.state.setDriveControlState(DriveControlState.FIELD_RELATIVE);
    Robot.shooterLimelight.setHumanPipelineActive();
  }
}
