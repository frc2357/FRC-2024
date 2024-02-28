package frc.robot.commands.state;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.state.RobotState.NoteState;

public class SetNoteState extends Command {
  private NoteState m_state;

  public SetNoteState(NoteState state) {
    m_state = state;
  }

  @Override
  public void initialize() {
    Robot.state.setNoteState(m_state);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
