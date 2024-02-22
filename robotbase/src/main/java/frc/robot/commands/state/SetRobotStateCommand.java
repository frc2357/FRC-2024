package frc.robot.commands.state;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.state.RobotState;

public class SetRobotStateCommand extends Command {
  private RobotState.State m_state;

  public SetRobotStateCommand(RobotState.State state) {
    m_state = state;
  }

  @Override
  public void initialize() {
    Robot.state.setState(m_state);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
