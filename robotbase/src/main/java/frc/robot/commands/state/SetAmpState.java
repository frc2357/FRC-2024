package frc.robot.commands.state;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.state.RobotState.AmpScoreState;

public class SetAmpState extends Command {
   private AmpScoreState m_state;

   public SetAmpState(AmpScoreState state) {
      m_state = state;
   }

   @Override
   public void initialize() {
      Robot.state.setAmpScoreState(m_state);
   }

   @Override
   public boolean isFinished() {
      return true;
   }
}
