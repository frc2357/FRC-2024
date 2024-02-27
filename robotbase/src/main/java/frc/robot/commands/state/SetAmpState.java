package frc.robot.commands.state;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.state.RobotState.AmpState;

public class SetAmpState extends Command {
  private AmpState m_state;

  public SetAmpState(AmpState state) {
    m_state = state;
  }

  @Override
  public void initialize() {
    Robot.state.setAmpState(m_state);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
