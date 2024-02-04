package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.state.RobotState.DriveControlState;

public class DriveToApriltagCommand extends Command {
  private double m_tyOffset;
  private double m_txOffset;
  private double m_endRotation;
  private int m_pipelineIndex;

  public DriveToApriltagCommand(
      double tyOffset, double txOffset, double endRotation, int pipelineIndex) {
    m_tyOffset = tyOffset;
    m_txOffset = txOffset;
    m_endRotation = endRotation;
    m_pipelineIndex = pipelineIndex;
    addRequirements(Robot.drive);
  }

  @Override
  public void initialize() {
    Robot.shooterLimelight.setPipeline(m_pipelineIndex);
    Robot.state.setDriveControlState(DriveControlState.ROBOT_RELATIVE);
  }
}
